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
from pr2_joints_for_base_movement_bhpn_drake_connection import Pr2JointsForBaseMovementBhpnDrakeConnection

interface_path = 'drake/examples/bhpn_drake_interface/'
interface_build_path = 'drake/bazel-bin/drake/examples/bhpn_drake_interface/'
interface_path_absolute = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/'
interface_build_path_absolute = '/Users/tristanthrush/research/mit/drake/bazel-bin/drake/examples/bhpn_drake_interface/'

class BhpnDrakeInterface:

    def __init__(
            self,
            robotName,
            world,
            bhpnRobotConf,
            robotFixed,
            bhpnObjectConfs,
            fixedObjects,
            replacementShapes = {} #{'objA': (shapes.readOff('/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/object_conversion_utils/drake_really_graspable_soda.off'), [1.57,3.14159,3.14159,-0.055,0.015,0.08], 0.05, 'purple')}
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
        self.getGrippedObjects = {}

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
                mass = 100.0
                if obj == 'objA' or obj == 'sodaA':
                    mass = 0.015
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
        objectFiles = {}
        #objectFiles['objA'] = 'drake/examples/bhpn_drake_interface/object_conversion_utils/objA.urdf '
        objectFiles['drake_table'] = 'drake/examples/bhpn_drake_interface/objects/drake_table.sdf '
        objectFiles['drake_soda'] = 'drake/examples/bhpn_drake_interface/objects/drake_soda.urdf '
        return objectFiles

    def createDrakeSimulationCommand(self):
        command = interface_build_path_absolute + 'simulation '
        command += str(self.robotConnection.getNumJoints()) + ' '
        for joint in self.robotConnection.getDrakeRobotConf(
                self.bhpnRobotConf).joint_position:
            command += str(joint) + ' '
        command += self.robotName + ' '
        command += self.robotConnection.getUrdfPath() + ' '
        for value in [0., 0., 0., 0., 0., 0.]: # TODO: fix! self.convertToDrakePose(self.initialBhpnRobotPose):
            command += str(value) + ' '
        command += str(self.robotFixed).lower() + ' '
        position_index_number = 0
        if not self.robotFixed:
            self.drakePoseIndices[self.robotName] = (position_index_number, position_index_number + 7)
            position_index_number += 7
        position_index_number += self.robotConnection.getNumJoints()
        for obj in self.objectFiles:
            command += self.objectFiles[obj]
            objPose = self.convertToDrakePose(self.bhpnObjectConfs[obj][obj][0])
            
            if obj == 'drake_table':
                objPose[2] -= .4095 #TODO: fix!
            if obj == 'drake_soda':
                objPose[2] -= .06 #TODO: fix!
            for value in objPose:
                    command += str(value) + ' '
            if obj in self.fixedObjects:
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
            state.joint_position = self.robotConnection.getJointList(bhpnPath[index])
            state.joint_velocity = [0]*state.num_joints
            state.joint_effort = [0]*state.num_joints
            #modify the plan so that drake wont stupidly take the long way from 0 to 2 pi for continuous joints
            for continuousJointIndex in self.robotConnection.getContinuousJointListIndices():
                if state.joint_position[continuousJointIndex] - lastJointValuesOnPath[continuousJointIndex] > math.pi:
                    state.joint_position[continuousJointIndex] -= 2*math.pi
                if state.joint_position[continuousJointIndex] - lastJointValuesOnPath[continuousJointIndex] < -1*math.pi:
                    state.joint_position[continuousJointIndex] += 2*math.pi
                assert abs(state.joint_position[continuousJointIndex] - lastJointValuesOnPath[continuousJointIndex]) <= math.pi + 0.01
            print 'joint_positions: ', state.joint_position
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
            
            #if an object is gripped relax threshold for the gripper
            gripped = self.robotConnection.getGrippedObjects(self.contactResults, self.getBhpnObjectConfs().keys())
            for hand, endEffector in self.robotConnection.getHandsToEndEffectors().items():
                print gripped[endEffector]
                if gripped[endEffector] != []:
                    for index in self.robotConnection.getDrakeHandJointIndices()[hand]:
                        self.robotConnection.setIgnoredMoveThreshold(index, True)
                else:
                    for index in self.robotConnection.getDrakeHandJointIndices()[hand]:
                        self.robotConnection.setIgnoredMoveThreshold(index, False)
            
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
        return obj in self.robotConnection.getGrippedObjects(self.contactResults, [obj])[self.robotConnection.getHandsToEndEffectors()[hand]]

    # BHPN Primitives that are robot-secific, and supported in the interface ##

    def robotSpecificPick(self, startConf, targetConf, hand, obj):
        return self.robotConnection.pick(startConf, targetConf, hand, obj, self)

    def robotSpecificPlace(self, params, useSM):
        raise NotImplementedError

    ######### LCM methods #####################################################

    def handleLcmSubscribers(self):
        while not self.handlerPoisonPill:
            self.lc.handle()

    def robotConfCallback(self, channel, data):
        self.drakeRobotConf = lcmt_robot_state.decode(data)
        self.bhpnRobotConf = self.robotConnection.getBhpnRobotConf(
            self.drakeRobotConf)
        self.robotConfUpdatedByDrake = True
     
    def objectPoseCallback(self, channel, data):
        msg = lcmt_viewer_draw.decode(data)
        for obj in self.bhpnObjectConfs:
            
            objPose = self.convertToBhpnPose(
            msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
            if obj == 'drake_table':
                #print 'z before: ', objPose.z
                objPose.z += .4095 #TODO: fix!
                #print 'z after: ', objPose.z
            if obj == 'drake_soda':
                objPose.z += .06 #TODO: fix!
            self.bhpnObjectConfs[obj] = objPose
            #print self.getBhpnObjectConfs()[obj].z

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
