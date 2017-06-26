import os
import multiprocessing
import threading
from geometry import shapes
import lcm
from drake import lcmt_robot_state
from operator import sub
import time

interface_path = '/home/tristanthrush/FakeDesktop/spartan/drake/drake/examples/bhpn_drake_interface/'
interface_build_path = '/home/tristanthrush/FakeDesktop/spartan/build/drake/drake/examples/bhpn_drake_interface/'

bhpnToDrakeRobotConfMappings = {'PR2' : [('pr2Torso', 1), ('pr2Head', 2), ('pr2RightArm', 5), ('pr2LeftArm', 5)], 'IIWA' : [('robotRightArm', 7)]}

robotInsertionArguments = {'PR2' : 'true 13 50000 300 300 300 300 300 300 300 300 300 300 300 300 5 5 5 5 5 5 5 5 5 5 5 5 5 7 7 7 7 7 7 7 7 7 7 7 7 7 /examples/PR2/pr2_fixed.urdf ', 'IIWA' : 'false 7 300 300 300 300 300 300 300 0 0 0 0 0 0 0 5 5 5 5 5 5 5 /manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf '}


class BhpnDrakeInterface:

	def __init__(self, robotName, world, robotConf, fixedRobot, objectConfs, fixedObjects):
		try:
			self.robotConfMapping = bhpnToDrakeRobotConfMappings[robotName]
			self.robotInsertionArguments = robotInsertionArguments[robotName]

		except KeyError:
			print "It appears that this robot is not supported in the bhpn to drake interface. The good news is that you can easily add support for BHPN control of any robot in Drake by adding some arguments to the dictonary at the top of this file."	

		self.numJoints = sum(map(lambda robotSection: robotSection[1], self.robotConfMapping))
		
		self.drakeRobotConf = None
		self.bhpnRobotConf = None

		#create lcm, which allows communication between drake and bhpn
		self.handlerPoisonPill = False
		self.lc = lcm.LCM()
		self.lc.subscribe('DRAKE_ROBOT_STATE', self.robotConfCallback)
		self.robotConfHandler = threading.Thread(target=self.handleRobotConfCallback)
		self.robotConfHandler.daemon = True
		self.robotConfHandler.start()	

		#start the thread with the drake simulation of the world
		self.drakeSimulationCommand = self.createDrakeSimulationCommand(world, fixedRobot, objectConfs, fixedObjects)
		self.drakeSimulation = multiprocessing.Process(target=os.system, args=[self.drakeSimulationCommand])
		self.drakeSimulation.daemon = True
		self.drakeSimulation.start()
		
		#wait for first message from drake about robot configuration
		while self.drakeRobotConf is None:
			time.sleep(0.1)	
		
		self.commandDrakeRobotConf(robotConf, 0.3)
		self.bhpnRobotConf = robotConf
		
		print 'initialized the bhpn-drake interface'

	def __del__(self):

		print 'attempting to terminate the bhpn-drake interface'

		self.handlerPoisonPill = True
		self.robotConfHandler.join()
		self.drakeSimulation.terminate()

		print 'terminated the bhpn-drake interface'

	def createDrakeSimulationCommand(self, world, fixedRobot, objectConfs, fixedObjects):

		def convertToDrakePose(pose):
			return pose.x, pose.y, pose.z, 0, 0, pose.theta

		def bhpnToDrakeObject(obj):
			path_to_objects = interface_path + 'object_conversion_utils/'
			shapes.writeOff(world.objectShapes[obj], path_to_objects + 'generated_bhpn_objects/' + obj + '.off')
			os.system(path_to_objects + 'bhpn_to_drake_object.py ' + path_to_objects + 'generated_bhpn_objects/' + obj + '.off')

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
				command += '/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.off.urdf '
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

		while max(map(abs, map(sub, msg.joint_position, self.drakeRobotConf.joint_position))) >= threshold:
			print 'moving drake robot to bhpn commanded configuration'
			self.lc.publish('BHPN_ROBOT_STATE_COMMAND', msg.encode())
			time.sleep(0.1)

	def getBhpnRobotConf(self):
		while self.bhpnRobotConf is None:
			print 'waiting for drake simulation to start publishing robot config'
		return self.bhpnRobotConf.copy()

	def getDrakeSimulationWorld(self):
		return self.drakeSimulationWorld.copy()
		
	def getDrakeSimulationFixedObjects(self):
		return self.drakeSimulationFixedObjects.copy()

	def robotConfCallback(self, channel, data):
		self.drakeRobotConf = lcmt_robot_state.decode(data)
		if self.bhpnRobotConf is not None:
			updatedNumJoints = 0
			for robotSection in self.robotConfMapping:
				self.bhpnRobotConf.set(robotSection[0], list(self.drakeRobotConf.joint_position[updatedNumJoints:robotSection[1]]) + list(self.bhpnRobotConf[robotSection[0]][robotSection[1]:-1]))
				updatedNumJoints += robotSection[1]
		

	def handleRobotConfCallback(self):
		while not self.handlerPoisonPill:
			self.lc.handle()

	

	
			
		
		
		





