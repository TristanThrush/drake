import os
import multiprocessing
import subprocess
import threading
from geometry import shapes, hu
from robot import conf
import lcm
from drake import lcmt_robot_state, lcmt_viewer_draw, lcmt_contact_results_for_viz
from bot_core import robot_state_t
from robotlocomotion import robot_plan_t
from operator import sub
import time
import bhpn_to_drake_object
import atexit
import math
import pydrake
import numpy as np
import subprocess
import signal

interface_path = 'drake/examples/bhpn_drake_interface/'
interface_build_path = 'drake/bazel-bin/drake/examples/bhpn_drake_interface/'
interface_path_absolute = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/'
interface_build_path_absolute = '/Users/tristanthrush/research/mit/drake/bazel-bin/drake/examples/bhpn_drake_interface/'

class RobotBhpnDrakeConnection:

    def __init__(self, bhpnRobotConf):
        self.bhpnRobotConf = bhpnRobotConf.copy()

    def toBhpnRobotConf(self, drakeRobotConf):
        raise NotImplementedError

    def toDrakeRobotConf(self, bhpnRobotConf):
        raise NotImplementedError

    def getUrdfPath(self):
        raise NotImplementedError

    def getNumJoints(self):
        raise NotImplementedError


class Pr2JointsForBaseMovementBhpnDrakeConnection(RobotBhpnDrakeConnection):

    def __init__(self, bhpnRobotConf):
        RobotBhpnDrakeConnection.__init__(self, bhpnRobotConf)
        self.robotName = 'pr2'
        self.numJoints = 24
        self.urdfPath = 'drake/examples/PR2/pr2_with_joints_for_base_movement_and_limited_gripper_movement.urdf'
        self.moveThreshold = np.array([.0075, .0075, .0015, .015, .015, .015, .015, .015, .015, .015, .015, .015, .015, .050, .050, .015, .015, .015, .015, .015, .015, .015, .050, .050])
        self.moveThreshold *= 4

    def toBhpnRobotConf(self, drakeRobotConf):
        drake_joints = drakeRobotConf.joint_position
        mapping = {'pr2Base': drake_joints[0:3],
                   'pr2Torso': drake_joints[3:4],
                   'pr2Head': drake_joints[4:6],
                   'pr2RightArm': drake_joints[6:13],
                   'pr2RightGripper': [np.sqrt(drake_joints[13:14][0]/100.0)],
                   'pr2LeftArm': drake_joints[15:22],
                   'pr2LeftGripper': [np.sqrt(drake_joints[22:23][0]/100.0)]}
        for k, v in mapping.items():
            self.bhpnRobotConf = self.bhpnRobotConf.set(k, list(v))
        return self.bhpnRobotConf.copy()

    def toDrakeRobotConf(self, bhpnRobotConf):
        msg = lcmt_robot_state()
        msg.timestamp = time.time() * 1000000
        msg.num_joints = self.numJoints
        msg.joint_position = self.toJointList(bhpnRobotConf)
        msg.joint_robot = [0] * msg.num_joints
        msg.joint_name = [''] * msg.num_joints
        msg.joint_velocity = [0.0] * msg.num_joints
        print 'msg: ', msg
        return msg
 
    def toJointList(self, bhpnRobotConf):
        return bhpnRobotConf['pr2Base']\
            + bhpnRobotConf['pr2Torso']\
            + bhpnRobotConf['pr2Head']\
            + bhpnRobotConf['pr2RightArm']\
            + [min([0.5, 100*bhpnRobotConf['pr2RightGripper'][0]**2])] * 2\
            + bhpnRobotConf['pr2LeftArm']\
            + [min([0.5, 100*bhpnRobotConf['pr2LeftGripper'][0]**2])] * 2

    def getJointListNames(self):
        return ['x', 'y', 'theta', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint']
    
    def getUrdfPath(self):
        return self.urdfPath

    def getNumJoints(self):
        return self.numJoints

    def getMoveThreshold(self):
        return self.moveThreshold.copy()

    def handToEndEffector(self):
        return {'left':'l_gripper_palm_link', 'right':'r_gripper_palm_link'}

    def getContinuousJointListIndices(self):
        return [5, 7, 9, 14, 16, 18]

    def grippedObjects(self, contact_results, objectsToCheck):
        gripper_end_effector_to_gripped_objects = {'l_gripper_palm_link':[], 'r_gripper_palm_link':[]}
        necessary_collisions_for_l_gripper_grip = set(['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
        necessary_collisions_for_r_gripper_grip = set(['r_gripper_l_finger_tip_link'])#, 'r_gripper_r_finger_tip_link']) //TODO: figure out why this is.
        necessary_collisions_combined = set()
        necessary_collisions_combined.update(necessary_collisions_for_l_gripper_grip)
        necessary_collisions_combined.update(necessary_collisions_for_r_gripper_grip)
        for obj in objectsToCheck:
            objContactsList = map(lambda contact_info: contact_info.body2_name, filter(lambda contact_info: contact_info.body1_name == obj and (contact_info.body2_name in necessary_collisions_combined), contact_results.contact_info))
            objContactsList += map(lambda contact_info: contact_info.body1_name, filter(lambda contact_info: contact_info.body2_name == obj and (contact_info.body1_name in necessary_collisions_combined), contact_results.contact_info))
            objContacts = set(objContactsList)
            if necessary_collisions_for_l_gripper_grip.issubset(objContacts):
                gripper_end_effector_to_gripped_objects['l_gripper_palm_link'] += [obj]
            if necessary_collisions_for_r_gripper_grip.issubset(objContacts):
                gripper_end_effector_to_gripped_objects['r_gripper_palm_link'] += [obj]

        return gripper_end_effector_to_gripped_objects
        

class BhpnDrakeInterface:

    def __init__(
            self,
            robotName,
            world,
            bhpnRobotConf,
            robotFixed,
            bhpnObjectConfs,
            fixedObjects,
            replacementShapes = {'objA': (shapes.readOff('/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/object_conversion_utils/drake_graspable_soda.off'), [1.57,3.14159,3.14159,-0.05,0.03,0.04], 2.0, 'purple')}
            ):
        self.robotName = robotName.lower()
        supportedRobots = {'pr2': Pr2JointsForBaseMovementBhpnDrakeConnection}

        try:
            self.robotConnection = supportedRobots[self.robotName](
                bhpnRobotConf)

        except KeyError:
            print "It appears that this robot is not supported in the bhpn to drake interface."
        print 'confs: ', bhpnObjectConfs
        self.lastCommandedDrakeRobotConf = None
        self.replacementShapes = replacementShapes
        self.drakePoseIndices = {}
        self.grippedObjects = {}

        self.world = world.copy()
        self.bhpnRobotConf = bhpnRobotConf.copy()
        self.initialBhpnRobotPose = self.bhpnRobotConf.basePose()
        self.robotFixed = robotFixed
        self.bhpnObjectConfs = bhpnObjectConfs.copy()
        self.fixedObjects = fixedObjects.copy()

        self.drakeRobotConf = None
        self.contactResults = None
        self.drakePosesXYZRPY = {}
        self.drakePosesXYZQ = {}
        self.robotConfUpdatedByDrake = False
        self.objectFiles = self.createDrakeObjectFiles()
        self.urdfToName = {
            urdf: name for name,
            urdf in self.objectFiles.iteritems()}
        self.urdfToName[interface_path +
                        self.robotConnection.getUrdfPath()] = self.robotName

        self.lc = lcm.LCM()
        self.lc.subscribe('ROBOT_STATE', self.robotConfCallback)
        self.lc.subscribe('DRAKE_VIEWER_DRAW', self.objectPoseCallback)
        self.lc.subscribe('CONTACT_RESULTS', self.contactResultsCallback)

        self.handlerPoisonPill = False
        self.robotConfHandler = threading.Thread(
            target=self.handleLcmSubscribers)
        self.robotConfHandler.daemon = True
        self.robotConfHandler.start()
        
        self.drakeSimulation = subprocess.Popen(
            'cd ' +
            interface_build_path_absolute +
            '; ' +
            'exec ' +
            self.createDrakeSimulationCommand(),
            shell=True)
        
        while self.robotConfUpdatedByDrake is False or self.contactResults is None:
            time.sleep(0.1)

        atexit.register(self.release)
        print 'Initialized the bhpn-drake interface.'

    ############# Initialization/Destruction methods ##########################

    def release(self):
        print 'Attempting to terminate the bhpn-drake interface.'
        self.handlerPoisonPill = True
        self.robotConfHandler.join()
        self.drakeSimulation.terminate()
        returnCode = self.drakeSimulation.wait()
        print 'return code of drake simulation: ', returnCode
        print 'Terminated the bhpn-drake interface.'

    def createDrakeObjectFiles(self):
        objectFiles = {}
        for obj in self.bhpnObjectConfs:
            if obj not in self.replacementShapes:
                shape = self.world.objectShapes[obj]
                transform = [0,0,0,0,0,0]
                mass = 6.0
                color = 'grey'
            else:
                shape = self.replacementShapes[obj][0]
                transform = self.replacementShapes[obj][1]
                mass = self.replacementShapes[obj][2]
                color = self.replacementShapes[obj][3]
            path_to_objects = interface_path_absolute + 'object_conversion_utils/'
            shapes.writeOff(
                shape,
                path_to_objects +
                'generated_bhpn_objects/' +
                obj +
                '.off')
            bhpn_to_drake_object.convert(
                path_to_objects + 'generated_bhpn_objects/' + obj + '.off', transform, mass, color)
            objectFiles[obj] = 'drake/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.urdf '
        return objectFiles

    def createDrakeSimulationCommand(self):
        command = interface_build_path_absolute + 'simulation '
        command += str(self.robotConnection.getNumJoints()) + ' '
        for joint in self.robotConnection.toDrakeRobotConf(
                self.bhpnRobotConf).joint_position:
            command += str(joint) + ' '
        command += self.robotName + ' '
        command += self.robotConnection.getUrdfPath() + ' '
        for value in self.convertToDrakePose(self.initialBhpnRobotPose):
            command += str(value) + ' '
        command += str(self.robotFixed).lower() + ' '
        position_index_number = 0
        if not self.robotFixed:
            self.drakePoseIndices[self.robotName] = (position_index_number, position_index_number + 7)
            position_index_number += 7
        position_index_number += self.robotConnection.getNumJoints()
        for obj in self.objectFiles:
            command += self.objectFiles[obj]
            for value in self.convertToDrakePose(
                    self.bhpnObjectConfs[obj][obj][0]):
                command += str(value) + ' '
            if obj in self.fixedObjects:# or obj == 'tableIkea1': #TODO: remove after youve had your fun
                command += 'true '
            else:
                command += 'false '
                self.drakePoseIndices[obj] = (position_index_number, position_index_number + 7)
                position_index_number += 7
        print 'Created drake simulation command: ', command
        return command

    ######### Simulation command methods ######################################

    def encodeDrakeRobotPlanFromBhpnPath(self, bhpnPath, time):
        assert len(time) == len(bhpnPath)
        print 'encoding path: '
        for jointConf in bhpnPath:
            jointConf.prettyPrint()
        plan = robot_plan_t()
        plan.utime = 0
        plan.robot_name = self.robotName 
        plan.num_states = len(time)
        plan.plan_info = [1]*plan.num_states #TODO: figure out what this is
        lastJointValuesOnPath = self.drakeRobotConf.joint_position #
        for index in range(len(bhpnPath)):
            state = robot_state_t()
            state.utime = time[index]
            state.num_joints = self.robotConnection.getNumJoints()
            state.joint_name = self.robotConnection.getJointListNames()
            state.joint_position = self.robotConnection.toJointList(bhpnPath[index])
            state.joint_velocity = [0]*state.num_joints
            state.joint_effort = [0]*state.num_joints
            #modify the plan so that drake wont stupidly take the long way from 0 to 2 pi for continuous joints
            for continuousJointIndex in self.robotConnection.getContinuousJointListIndices():
                if state.joint_position[continuousJointIndex] - lastJointValuesOnPath[continuousJointIndex] > math.pi:
                    state.joint_position[continuousJointIndex] -= 2*math.pi
                if state.joint_position[continuousJointIndex] - lastJointValuesOnPath[continuousJointIndex] < -1*math.pi:
                    state.joint_position[continuousJointIndex] += 2*math.pi
            lastJointValuesOnPath = state.joint_position
            plan.plan.append(state)
        plan.num_grasp_transitions = 0
        plan.left_arm_control_type = plan.POSITION
        plan.right_arm_control_type = plan.POSITION
        plan.left_leg_control_type = plan.POSITION
        plan.right_leg_control_type = plan.POSITION
        plan.num_bytes = 0
        return plan

    def commandDrakeRobotPlan(self, plan):
        print "Attempting to follow plan"
        self.lc.publish('ROBOT_PLAN', plan.encode())
        #TODO: think about the best way to consider this plan finished
        
        while np.max(np.abs(np.array(plan.plan[-1].joint_position) - np.array(self.drakeRobotConf.joint_position)) - self.robotConnection.getMoveThreshold()) > 0:
            print np.abs(np.array(plan.plan[-1].joint_position) - np.array(self.drakeRobotConf.joint_position)) - self.robotConnection.getMoveThreshold()
            time.sleep(0.1)
        
        #time.sleep(plan.plan[-1].utime/20000.0)
        print "Done following plan"

    ######### Getter methods ##################################################

    def getBhpnRobotConf(self):
        return self.bhpnRobotConf.copy()

    def getBhpnObjectConfs(self):
        return self.bhpnObjectConfs.copy()

    def isGripped(self, hand, obj):
        return obj in self.robotConnection.grippedObjects(self.contactResults, ['objA'])[self.robotConnection.handToEndEffector()[hand]]

    ######### LCM methods #####################################################

    def handleLcmSubscribers(self):
        while not self.handlerPoisonPill:
            self.lc.handle()

    def robotConfCallback(self, channel, data):
        self.drakeRobotConf = lcmt_robot_state.decode(data)
        self.bhpnRobotConf = self.robotConnection.toBhpnRobotConf(
            self.drakeRobotConf)
        self.robotConfUpdatedByDrake = True
     
    def objectPoseCallback(self, channel, data):
        msg = lcmt_viewer_draw.decode(data)
        for obj in self.bhpnObjectConfs:
            self.bhpnObjectConfs[obj] = self.convertToBhpnPose(
            msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])

        #print self.bhpnObjectConfs
        for obj in msg.link_name:
            self.drakePosesXYZRPY[obj] = list(msg.position[msg.link_name.index(
                obj)]) + self.convertFromQuaternionToRPY(msg.quaternion[msg.link_name.index(obj)])
            self.drakePosesXYZQ[obj] = list(msg.position[msg.link_name.index(
                obj)]) + list(msg.quaternion[msg.link_name.index(obj)])
        self.drakePosesXYZRPY[self.robotName] = [
            0, 0, 0, 0, 0, 0]  # TODO: fix this!
        

    def contactResultsCallback(self, channel, data):
        self.contactResults = lcmt_contact_results_for_viz.decode(data)            
        
    ######### Conversion utility methods ######################################

    def convertToBhpnPose(self, position, orientation):
        return hu.Pose(
            position[0],
            position[1],
            position[2],
            self.convertFromQuaternionToRPY(orientation)[2])
            
    def convertToDrakePose(self, pose):
        return [pose.x, pose.y, pose.z, 0, 0, pose.theta]

    def convertFromQuaternionToRPY(self, quaternion):
        w = quaternion[0]
        x = quaternion[1]
        y = quaternion[2]
        z = quaternion[3]

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        r = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        p = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        y = math.atan2(t3, t4)
        return [r, p, y]

    def linkToLinkDrakeTransform(self, link1_name, link2_name):
        return np.array(self.drakePosesXYZQ[link1_name]) - np.array(self.drakePosesXYZQ[link2_name])

    ######### IK planner methods  ############################################

    def simpleIkPlanner(self, constraints):
        world_tree = self.kinematicsSnapshot()
        q_seed = np.array(self.drakeRobotConf.joint_position)
        options = ik.IKoptions(world_tree)
        return ik.InverseKinTraj(
            world_tree,
            q_seed,
            q_seed,
            constraints,
            options)

    def kinematicsSnapshot(self):
        world_tree = pydrake.rbtree.RigidBodyTree()
        for urdf_file, name in self.urdfToName.items():
            urdf_string = open(urdf_file).read()
            base_dir = os.path.dirname(urdf_file)
            package_map = pydrake.rbtree.PackageMap()
            pose = self.drakePosesXYZRPY[name]
            weld_frame = pydrake.rbtree.RigidBodyFrame(
                "world", None, np.array(pose[0:3]), np.array(pose[3:6]))
            # TODO: ask whether making objects fixed and not including current
            # forces will reduce robustness of ik planner (ex: does ik planner
            # know that an object will fall to ground after a certian time,
            # freeing up space?)
            floating_base_type = pydrake.rbtree.kFixed
            pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
                urdf_string,
                package_map,
                base_dir,
                floating_base_type,
                weld_frame,
                world_tree)
        return world_tree
