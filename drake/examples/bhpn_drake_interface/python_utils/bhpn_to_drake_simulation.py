import os
import multiprocessing
import subprocess
import threading
from geometry import shapes, hu
from robot import conf
import lcm
from lcmt_robot_state import lcmt_robot_state
from lcmt_viewer_draw import lcmt_viewer_draw
from lcmt_contact_results_for_viz import lcmt_contact_results_for_viz
from operator import sub
import time
import bhpn_to_drake_object
import atexit
import math
import pydrake
import numpy as np

interface_path = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/'
interface_build_path = '/Users/tristanthrush/research/mit/drake/bazel-bin/drake/examples/bhpn_drake_interface/'

#'IIWA' : [('robotRightArm', 7)]

#'IIWA' : 'iiwa /manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf '


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


class Pr2BhpnDrakeConnection(RobotBhpnDrakeConnection):

    def __init__(self, bhpnRobotConf):
        RobotBhpnDrakeConnection.__init__(self, bhpnRobotConf)
        self.robotName = 'pr2'
        self.numJoints = 21
        self.urdfPath = '/examples/PR2/pr2_fixed.urdf'
        self.moveThreshold = np.array([.04, .4, .4, .4, .4, .4, .4, .4, .4, .4, .1, .1, .4, .4, .4, .4, .4, .4, .4, .1, .1])

    def toBhpnRobotConf(self, drakeRobotConf):
        drake_joints = drakeRobotConf.joint_position
        mapping = {'pr2Torso': drake_joints[0:1],
                   'pr2Head': drake_joints[1:3],
                   'pr2RightArm': drake_joints[3:10],
                   'pr2RightGripper': [np.sqrt(drake_joints[10:11][0]/100.0)],
                   'pr2LeftArm': drake_joints[12:19],
                   'pr2LeftGripper': [np.sqrt(drake_joints[19:20][0]/100.0)]}
        for k, v in mapping.items():
            self.bhpnRobotConf = self.bhpnRobotConf.set(k, list(v))
        return self.bhpnRobotConf.copy()

    def toDrakeRobotConf(self, bhpnRobotConf):
        msg = lcmt_robot_state()
        msg.timestamp = time.time() * 1000000
        msg.num_joints = self.numJoints
        msg.joint_position = bhpnRobotConf['pr2Torso']\
            + bhpnRobotConf['pr2Head']\
            + bhpnRobotConf['pr2RightArm']\
            + [min([0.5, 100*bhpnRobotConf['pr2RightGripper'][0]**2])] * 2\
            + bhpnRobotConf['pr2LeftArm']\
            + [min([0.5, 100*bhpnRobotConf['pr2LeftGripper'][0]**2])] * 2
        msg.joint_robot = [0] * msg.num_joints
        msg.joint_name = [''] * msg.num_joints
        msg.joint_velocity = [0.0] * msg.num_joints
        return msg

    def getUrdfPath(self):
        return self.urdfPath

    def getNumJoints(self):
        return self.numJoints

    def getMoveThreshold(self):
        return self.moveThreshold.copy()

    def grippedObjects(self):
        pass
        

class BhpnDrakeInterface:

    def __init__(
            self,
            robotName,
            world,
            bhpnRobotConf,
            robotFixed,
            bhpnObjectConfs,
            fixedObjects,
            perfectControlAndObjectHolding = True
            ):
        self.robotName = robotName.lower()
        supportedRobots = {'pr2': Pr2BhpnDrakeConnection}

        try:
            self.robotConnection = supportedRobots[self.robotName](
                bhpnRobotConf)

        except KeyError:
            print "It appears that this robot is not supported in the bhpn to drake interface."

        self.perfectControlAndObjectHolding = perfectControlAndObjectHolding
        self.world = world.copy()
        self.bhpnRobotConf = bhpnRobotConf.copy()
        self.initialBhpnRobotPose = self.bhpnRobotConf.basePose()
        self.robotFixed = robotFixed
        self.bhpnObjectConfs = bhpnObjectConfs.copy()
        self.fixedObjects = fixedObjects.copy()

        self.drakeRobotConf = None
        self.contactResults = None
        self.drakePosesXYZRPY = {}
        self.robotConfUpdatedByDrake = False
        self.objectFiles = self.createDrakeObjectFiles()
        self.urdfToName = {
            urdf: name for name,
            urdf in self.objectFiles.iteritems()}
        self.urdfToName[interface_path +
                        self.robotConnection.getUrdfPath()] = self.robotName

        self.lc = lcm.LCM()
        self.lc.subscribe('DRAKE_ROBOT_STATE', self.robotConfCallback)
        self.lc.subscribe('DRAKE_VIEWER_DRAW', self.objectPoseCallback)
        self.lc.subscribe('CONTACT_RESULTS', self.contactResultsCallback)

        self.handlerPoisonPill = False
        self.robotConfHandler = threading.Thread(
            target=self.handleLcmSubscribers)
        self.robotConfHandler.daemon = True
        self.robotConfHandler.start()

        self.drakeSimulation = subprocess.Popen(
            'cd ' +
            interface_build_path +
            '; ' +
            'exec ' +
            self.createDrakeSimulationCommand(),
            shell=True)

        while self.robotConfUpdatedByDrake is False:
            time.sleep(0.1)

        atexit.register(self.release)
        print 'Initialized the bhpn-drake interface.'

    ############# Initialization/Destruction methods ##########################

    def release(self):
        print 'Attempting to terminate the bhpn-drake interface.'
        self.handlerPoisonPill = True
        self.robotConfHandler.join()
        self.drakeSimulation.terminate()
        returncode = self.drakeSimulation.wait()
        print "Returncode of simulation: ", returncode
        print 'Terminated the bhpn-drake interface.'

    def createDrakeObjectFiles(self):
        objectFiles = {}
        for obj in self.bhpnObjectConfs:
            path_to_objects = interface_path + 'object_conversion_utils/'
            shapes.writeOff(
                self.world.objectShapes[obj],
                path_to_objects +
                'generated_bhpn_objects/' +
                obj +
                '.off')
            bhpn_to_drake_object.convert(
                path_to_objects + 'generated_bhpn_objects/' + obj + '.off')
            objectFiles[obj] = '/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.urdf '
        return objectFiles

    def createDrakeSimulationCommand(self):
        command = interface_build_path + 'simulation '
        command += str(self.perfectControlAndObjectHolding).lower() + ' '
        command += str(self.robotConnection.getNumJoints()) + ' '
        for joint in self.robotConnection.toDrakeRobotConf(
                self.bhpnRobotConf).joint_position:
            command += str(joint) + ' '
        command += self.robotName + ' '
        command += self.robotConnection.getUrdfPath() + ' '
        for value in self.convertToDrakePose(self.initialBhpnRobotPose):
            command += str(value) + ' '
        command += str(self.robotFixed).lower() + ' '
        for obj in self.objectFiles:
            command += self.objectFiles[obj]
            for value in self.convertToDrakePose(
                    self.bhpnObjectConfs[obj][obj][0]):
                command += str(value) + ' '
            if obj in self.fixedObjects: # or obj == 'tableIkea1': #TODO: remove after youve had your fun
                command += 'true '
            else:
                command += 'false '
        print 'Created drake simulation command: ', command
        return command

    ######### Simulation command methods ######################################

    def commandDrakeRobotConf(self, bhpnConf, waitTime=10):
        print "COMMAND BHPN: ", bhpnConf
        msg = self.robotConnection.toDrakeRobotConf(bhpnConf)
        print 'Attempting to move drake robot to bhpn commanded configuration.'
        self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())
        startTime = time.time()
        print "COMMAND DRAKE: ", msg.joint_position
        if self.perfectControlAndObjectHolding: time.sleep(0.1) #so you have time to see the motion
        while np.max(np.abs(np.array(msg.joint_position) - np.array(self.drakeRobotConf.joint_position)) - self.robotConnection.getMoveThreshold()) > 0: # and time.time() - startTime < waitTime:
            print time.time() - startTime
            print np.abs(np.array(msg.joint_position) - np.array(self.drakeRobotConf.joint_position)) - self.robotConnection.getMoveThreshold()
            time.sleep(0.1)
            self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())
        print 'Moved drake robot to bhpn commanded configuration.'

    ######### Getter methods ##################################################

    def getBhpnRobotConf(self):
        return self.bhpnRobotConf.copy()

    def getBhpnObjectConfs(self):
        return self.bhpnObjectConfs.copy()

    def isGripped():
        pass

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
            self.drakePosesXYZRPY[obj] = list(msg.position[msg.link_name.index(
                obj)]) + self.convertFromQuaternionToRPY(msg.quaternion[msg.link_name.index(obj)])
            self.bhpnObjectConfs[obj] = self.convertToBhpnPose(
                msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
        self.drakePosesXYZRPY[self.robotName] = [
            0, 0, 0, 0, 0, 0]  # TODO: fix this!

    def contactResultsCallback(self, channel, data):
        self.contactResults = lcmt_contact_results_for_viz.decode(data)
        #TODO: remove the following
        #print "active"
        for info in self.contactResults.contact_info:
            #if info.body1_name == 'objA' and info.body2_name != 'tableIkea1':
            if ('gripper' in info.body1_name and 'obj' in info.body2_name) or ('gripper' in info.body2_name and 'obj' in info.body1_name):
                for i in range(1000000):
                    print 'Contact: ', info.body1_name, info.body2_name
            if 'tip' in info.body1_name:
                print info.body1_name

                
        
    ######### Conversion utility methods ######################################

    def convertToBhpnPose(self, position, orientation):
        return hu.Pose(
            position[0],
            position[1],
            position[2],
            2 *
            math.acos(
                orientation[0]))

    def convertToDrakePose(self, pose):
        return [pose.x, pose.y, pose.z, 0, 0, pose.theta]

    def convertFromQuaternionToRPY(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        r = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        p = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        y = math.degrees(math.atan2(t3, t4))
        return [r, p, y]

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
