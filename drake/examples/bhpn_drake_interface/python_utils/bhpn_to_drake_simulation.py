import os
import multiprocessing
import subprocess
import threading
from geometry import shapes, hu
from robot import conf
import lcm
from lcmt_robot_state import lcmt_robot_state
from lcmt_viewer_draw import lcmt_viewer_draw
from operator import sub
import time
import bhpn_to_drake_object
import atexit
import math

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
    
    def getInsertionArguments(self):
        raise NotImplementedError

    def getNumJoints(self):
        raise NotImplementedError


class Pr2BhpnDrakeConnection(RobotBhpnDrakeConnection):

    def __init__(self, bhpnRobotConf):
        RobotBhpnDrakeConnection.__init__(self, bhpnRobotConf)
        self.robotName = 'PR2'
        self.numJoints = 21
        self.insertionArguments = 'pr2 /examples/PR2/pr2_fixed.urdf '

    def toBhpnRobotConf(self, drakeRobotConf):
        drake_joints = drakeRobotConf.joint_position
        mapping = {'pr2Torso': drake_joints[0:1],
                   'pr2Head': drake_joints[1:3],
                   'pr2RightArm': drake_joints[3:10],
                   'pr2RightGripper': drake_joints[10:11],
                   'pr2LeftArm': drake_joints[12:19],
                   'pr2LeftGripper': drake_joints[19:20]}
        for k, v in mapping.items():
            self.bhpnRobotConf = self.bhpnRobotConf.set(k, list(v))
        return self.bhpnRobotConf.copy()

    def toDrakeRobotConf(self, bhpnRobotConf):
        msg = lcmt_robot_state()
	msg.timestamp = time.time()*1000000
        msg.num_joints = self.numJoints
        msg.joint_position = bhpnRobotConf['pr2Torso']\
                            + bhpnRobotConf['pr2Head']\
                            + bhpnRobotConf['pr2RightArm']\
                            + bhpnRobotConf['pr2RightGripper']*2\
                            + bhpnRobotConf['pr2LeftArm']\
                            + bhpnRobotConf['pr2LeftGripper']*2
        msg.joint_robot = [0]*msg.num_joints
	msg.joint_name = ['']*msg.num_joints
	msg.joint_velocity = [0.0]*msg.num_joints
        return msg
    
    def getInsertionArguments(self):
        return self.insertionArguments

    def getNumJoints(self):
        return self.numJoints


class BhpnDrakeInterface:
  
  	def __init__(self, robotName, world, robotConf, fixedRobot, objectConfs, fixedObjects):
                supportedRobots = {'PR2': Pr2BhpnDrakeConnection}

                try:
		    self.robotConnection = supportedRobots[robotName](robotConf)
        
		except KeyError:
			print "It appears that this robot is not supported in the bhpn to drake interface."
		
		self.drakeRobotConf = None
		self.bhpnRobotConf = robotConf.copy() #the robot conf from the drake simulation, converted to the appropriate bhpn object type
                self.robotConfUpdatedByDrake = False
                self.bhpnObjectConfs = objectConfs #the object confs (aka poses) from the drake simulation, converted to the appropriate bbhpn object type

		#create lcm, which allows communication between drake and bhpn
		self.lc = lcm.LCM()

                #this is for controlling the robot 
		self.lc.subscribe('DRAKE_ROBOT_STATE', self.robotConfCallback)
                #This is for reciving the poses of all of the objects in the world. 
                #It piggybacks off of the messages sent by the drake simulator for the drake visualizer.
                self.lc.subscribe('DRAKE_VIEWER_DRAW', self.objectPoseCallback)
                #start a thread that handles the lcm subscribers
                self.handlerPoisonPill = False
                self.robotConfHandler = threading.Thread(target=self.handleLcmSubscribers)
		self.robotConfHandler.daemon = True
		self.robotConfHandler.start()	
               
		#start a subprocess with the drake simulation of the world
		self.drakeSimulationCommand = self.createDrakeSimulationCommand(world, fixedRobot, objectConfs, fixedObjects)
		self.drakeSimulation = subprocess.Popen('cd ' + interface_build_path + '; ' + 'exec '+self.drakeSimulationCommand, shell=True)		
		#wait for first message from drake about robot configuration
		while self.robotConfUpdatedByDrake is False:
			time.sleep(0.1)	
				
                atexit.register(self.release)
		print 'Initialized the bhpn-drake interface.'

	def release(self):

		print 'Attempting to terminate the bhpn-drake interface.'

		self.handlerPoisonPill = True
		self.robotConfHandler.join()
		self.drakeSimulation.terminate()
                returncode = self.drakeSimulation.wait()
                print "Returncode of simulation: ", returncode

		print 'Terminated the bhpn-drake interface.'

	def createDrakeSimulationCommand(self, world, fixedRobot, objectConfs, fixedObjects):                

		def convertToDrakePose(pose):
			return pose.x, pose.y, pose.z, 0, 0, pose.theta

		def bhpnToDrakeObject(obj):
			path_to_objects = interface_path + 'object_conversion_utils/'
			shapes.writeOff(world.objectShapes[obj], path_to_objects + 'generated_bhpn_objects/' + obj + '.off')
                	bhpn_to_drake_object.convert(path_to_objects + 'generated_bhpn_objects/' + obj + '.off')

    		command = interface_build_path + 'simulation '
                command += str(self.robotConnection.getNumJoints()) + ' '
                for joint in self.robotConnection.toDrakeRobotConf(self.bhpnRobotConf).joint_position:
                    command += str(joint) + ' '
		command += self.robotConnection.getInsertionArguments()
		command += '0 0 0 0 0 0 '
		command += str(fixedRobot).lower() + ' '

		for obj in world.objects:
			if obj in objectConfs:
				fixed = 'false '
				if obj in fixedObjects:
					fixed = 'true '
				bhpnToDrakeObject(obj)
				command += '/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.urdf '
				objDrakePose = convertToDrakePose(objectConfs[obj][obj][0])
				for value in objDrakePose:
					command += str(value) + ' '
				command += fixed
		print 'command: ', command
		return command
		

	def commandDrakeRobotConf(self, bhpnConf, threshold):
                msg = self.robotConnection.toDrakeRobotConf(bhpnConf)
                print 'Attempting to move drake robot to bhpn commanded configuration.'
                self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())
		while max(map(abs, map(sub, msg.joint_position, self.drakeRobotConf.joint_position))) >= threshold:
			time.sleep(0.1)
                        self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())           
                print 'Moved drake robot to bhpn commanded configuration.'


	def getBhpnRobotConf(self):
    		return self.bhpnRobotConf.copy()

	def getDrakeSimulationWorld(self):
		return self.drakeSimulationWorld.copy()
		
	def getDrakeSimulationFixedObjects(self):
		return self.drakeSimulationFixedObjects.copy()

	def robotConfCallback(self, channel, data):
		self.drakeRobotConf = lcmt_robot_state.decode(data)
                self.bhpnRobotConf = self.robotConnection.toBhpnRobotConf(self.drakeRobotConf)
                self.robotConfUpdatedByDrake = True
                               
        def getBhpnObjectConfs(self):
            return self.bhpnObjectConfs.copy()
                                
        def objectPoseCallback(self, channel, data):
                msg = lcmt_viewer_draw.decode(data)
                
                for obj in self.bhpnObjectConfs:
                    self.bhpnObjectConfs[obj] = self.convertToBhpnPose(msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
                                
        def convertToBhpnPose(self, position, orientation):
                return hu.Pose(position[0], position[1], position[2], 2*math.acos(orientation[0]))
		

	def handleLcmSubscribers(self):
		while not self.handlerPoisonPill:
			self.lc.handle()    
