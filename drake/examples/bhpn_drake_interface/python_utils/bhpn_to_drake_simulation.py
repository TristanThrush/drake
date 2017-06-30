import os
import multiprocessing
import subprocess
import threading
from geometry import shapes, hu
import lcm
from lcmt_robot_state import lcmt_robot_state
from lcmt_viewer_draw import lcmt_viewer_draw
from operator import sub
import time
import bhpn_to_drake_object
import atexit

interface_path = '/Users/tristanthrush/research/mit/drake/drake/examples/bhpn_drake_interface/'
interface_build_path = '/Users/tristanthrush/research/mit/drake/bazel-bin/drake/examples/bhpn_drake_interface/'

bhpnToDrakeRobotConfMappings = {'PR2' : [('pr2Torso', 1), ('pr2Head', 2), ('pr2RightArm', 5), ('pr2LeftArm', 5)], 'IIWA' : [('robotRightArm', 7)]}

robotInsertionArguments = {'PR2' : 'pr2 ' + '/examples/PR2/pr2_fixed.urdf ', 'IIWA' : 'iiwa /manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf '}


class BhpnDrakeInterface:
  
  	def __init__(self, robotName, world, robotConf, fixedRobot, objectConfs, fixedObjects):
		try:
			self.robotConfMapping = bhpnToDrakeRobotConfMappings[robotName]
			self.robotInsertionArguments = robotInsertionArguments[robotName]

		except KeyError:
			print "It appears that this robot is not supported in the bhpn to drake interface."

		self.numJoints = sum(map(lambda robotSection: robotSection[1], self.robotConfMapping))
		
		self.drakeRobotConf = None
		self.bhpnRobotConf = robotConf #the robot conf from the drake simulation, converted to the appropriate bhpn object type
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
		command += self.robotInsertionArguments
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
		msg = lcmt_robot_state()
		msg.timestamp = time.time()*1000000
		msg.num_joints = self.numJoints
		for robotSection in self.robotConfMapping:
			msg.joint_position += bhpnConf[robotSection[0]][:robotSection[1]]
		#currently the following are unused components of the message, as drake's robot state message is more general than I need
    		msg.joint_robot = [0]*self.numJoints
		msg.joint_name = ['']*self.numJoints
		msg.joint_velocity = [0.0]*self.numJoints
                
                print 'Attempting to move drake robot to bhpn commanded configuration.'
		while max(map(abs, map(sub, msg.joint_position, self.drakeRobotConf.joint_position))) >= threshold:
    			self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())
			time.sleep(0.01)
                    
                print 'Moved drake robot to bhpn commanded configuration.'


	def getBhpnRobotConf(self):
    		return self.bhpnRobotConf.copy()

	def getDrakeSimulationWorld(self):
		return self.drakeSimulationWorld.copy()
		
	def getDrakeSimulationFixedObjects(self):
		return self.drakeSimulationFixedObjects.copy()

	def robotConfCallback(self, channel, data):
		self.drakeRobotConf = lcmt_robot_state.decode(data)
		updatedNumJoints = 0
		for robotSection in self.robotConfMapping:
    			self.bhpnRobotConf = self.bhpnRobotConf.set(robotSection[0], list(self.drakeRobotConf.joint_position[updatedNumJoints:robotSection[1]]) + list(self.bhpnRobotConf[robotSection[0]][robotSection[1]-1:-1]))
			updatedNumJoints += robotSection[1]
                self.robotConfUpdatedByDrake = True

        def getBhpnObjectConfs(self):
            return self.bhpnObjectConfs.copy()
                                
        def objectPoseCallback(self, channel, data):
                msg = lcmt_viewer_draw.decode(data)
                
                for obj in self.bhpnObjectConfs:
                    self.bhpnObjectConfs[obj] = self.convertToBhpnPose(msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
                
        def convertToBhpnPose(self, position, orientation):
                return hu.Pose(position[0], position[1], position[2], orientation[3])
		

	def handleLcmSubscribers(self):
		while not self.handlerPoisonPill:
			self.lc.handle()    
        
        
 

