import os
import multiprocessing
from robot import rrt
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
#TODO: get rid of unused imports
from robot_bhpn_drake_connection import RobotBhpnDrakeConnection

class Pr2JointsForBaseMovementBhpnDrakeConnection(RobotBhpnDrakeConnection):

    def __init__(self, bhpnRobotConf):
        RobotBhpnDrakeConnection.__init__(self, bhpnRobotConf)
        self.robotName = 'pr2'
        self.numJoints = 24
        self.urdfPath = 'drake/examples/PR2/pr2_with_joints_for_base_movement_and_limited_gripper_movement.urdf'
        self.moveThreshold = np.array([.015, .015, .0015, .015, .015, .015, .015, .015, .015, .015, .015, .015, .015, .040, .040, .015, .015, .015, .015, .015, .015, .015, .040, .040])
        self.moveThreshold *= 4

    def getBhpnRobotConf(self, drakeRobotConf):
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

    def getDrakeRobotConf(self, bhpnRobotConf):
        msg = lcmt_robot_state()
        msg.timestamp = time.time() * 1000000
        msg.num_joints = self.numJoints
        msg.joint_position = self.getJointList(bhpnRobotConf)
        msg.joint_robot = [0] * msg.num_joints
        msg.joint_name = [''] * msg.num_joints
        msg.joint_velocity = [0.0] * msg.num_joints
        print 'msg: ', msg
        return msg
 
    def getJointList(self, bhpnRobotConf):
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

    def getHandsToEndEffectors(self):
        return {'left':'l_gripper_palm_link', 'right':'r_gripper_palm_link'}

    def getContinuousJointListIndices(self):
        return [2, 8, 10, 12, 17, 19, 21]

    def getGrippedObjects(self, contactResults, objectsToCheck):
        gripper_end_effector_to_gripped_objects = {'l_gripper_palm_link':[], 'r_gripper_palm_link':[]}
        necessary_collisions_for_l_gripper_grip = set(['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
        necessary_collisions_for_r_gripper_grip = set(['r_gripper_l_finger_tip_link', 'r_gripper_r_finger_tip_link'])
        necessary_collisions_combined = set()
        necessary_collisions_combined.update(necessary_collisions_for_l_gripper_grip)
        necessary_collisions_combined.update(necessary_collisions_for_r_gripper_grip)
        for obj in objectsToCheck:
            objContactsList = map(lambda contact_info: contact_info.body2_name, filter(lambda contact_info: contact_info.body1_name == obj and (contact_info.body2_name in necessary_collisions_combined), contactResults.contact_info))
            objContactsList += map(lambda contact_info: contact_info.body1_name, filter(lambda contact_info: contact_info.body2_name == obj and (contact_info.body1_name in necessary_collisions_combined), contactResults.contact_info))
            objContacts = set(objContactsList)
            if necessary_collisions_for_l_gripper_grip.issubset(objContacts):
                gripper_end_effector_to_gripped_objects['l_gripper_palm_link'] += [obj]
            if necessary_collisions_for_r_gripper_grip.issubset(objContacts):
                gripper_end_effector_to_gripped_objects['r_gripper_palm_link'] += [obj]
        
        if gripper_end_effector_to_gripped_objects['r_gripper_palm_link'] != []:
            objg = gripper_end_effector_to_gripped_objects['r_gripper_palm_link'][0]
            print 'grip: ', objg

        return gripper_end_effector_to_gripped_objects

    def pick(self, startConf, targetConf, hand, obj, bhpnDrakeInterfaceObj, timeout=60):
        #basic picking procedure. Not really that reactive (except it does ensure that the object is gripped hard enough). Can easily be made more reactive by taking more advantage of bhpnDrakeInterfaceObj's data from drake
        print 'startConf: '
        startConf.prettyPrint()
        print 'targetConf: '
        targetConf.prettyPrint()

        stepTime = 100000
        fingerOverlapLength = 0.05
        widthOpen = 0.07
        minWidthClosed = 0.035

        #move gripper close to object
        startConfOpen = startConf.set(startConf.robot.gripperChainNames[hand], [widthOpen])
        targetConfOpen = targetConf.set(targetConf.robot.gripperChainNames[hand], [widthOpen])
        path = rrt.interpolatePath([bhpnDrakeInterfaceObj.getBhpnRobotConf(), startConfOpen, targetConfOpen], 0.01)
        plan = bhpnDrakeInterfaceObj.encodeDrakeRobotPlanFromBhpnPath(path, [step*stepTime for step in range(len(path))])
        bhpnDrakeInterfaceObj.commandDrakeRobotPlan(plan)

        #move gripper right around object
        
        nConf = displaceHand(targetConfOpen, hand, dx=fingerOverlapLength, dz=0.0, nearTo=targetConfOpen)
        path = rrt.interpolatePath([bhpnDrakeInterfaceObj.getBhpnRobotConf(), nConf], 0.01)
        plan = bhpnDrakeInterfaceObj.encodeDrakeRobotPlanFromBhpnPath(path, [step*stepTime for step in range(len(path))])
        bhpnDrakeInterfaceObj.commandDrakeRobotPlan(plan)
        
        #move the fingers around the object and stop when we have gripped it hard enough or when the time to try is over ("hard enough" for this robot is defined by isGripped)
        
        startTime = time.time()
        while not bhpnDrakeInterfaceObj.isGripped(hand, obj):
            nConf = nConf.set(nConf.robot.gripperChainNames[hand], [max([minWidthClosed, nConf[nConf.robot.gripperChainNames[hand]][0] - 0.001])])
            plan = bhpnDrakeInterfaceObj.encodeDrakeRobotPlanFromBhpnPath([bhpnDrakeInterfaceObj.getBhpnRobotConf(), nConf], [0, 100000])
            bhpnDrakeInterfaceObj.commandDrakeRobotPlan(plan)
            if time.time() - startTime > timeout:
                return False
        
        #if the object has been gripped, then pick it up
        '''
        nConfUp = displaceHand(targetConfOpen, hand, dx=fingerOverlapLength, dz=0.05, nearTo=targetConfOpen)
        nConfUpClosed = nConfUp.set(nConfUp.robot.gripperChainNames[hand], nConf[nConfUp.robot.gripperChainNames[hand]])
        path = [bhpnDrakeInterfaceObj.getBhpnRobotConf(), nConfUpClosed]
        plan = bhpnDrakeInterfaceObj.encodeDrakeRobotPlanFromBhpnPath(path, [0, 500000])
        bhpnDrakeInterfaceObj.commandDrakeRobotPlan(plan)
        '''
        startConfClosed = startConf.set(startConf.robot.gripperChainNames[hand], nConf[nConf.robot.gripperChainNames[hand]])
        path = rrt.interpolatePath([bhpnDrakeInterfaceObj.getBhpnRobotConf(), startConfClosed], 0.1)
        plan = bhpnDrakeInterfaceObj.encodeDrakeRobotPlanFromBhpnPath(path, [step*stepTime for step in range(len(path))])
        bhpnDrakeInterfaceObj.commandDrakeRobotPlan(plan)
        
        return bhpnDrakeInterfaceObj.isGripped(hand, obj)

#TODO: get rid of this! it uses non-drake invKin
def displaceHand(conf, hand, dx=0.0, dy=0.0, dz=0.0,
                 zFrom=None, maxTarget=None, nearTo=None):
    print 'displaceHand'
    cart = conf.cartConf()
    robot = conf.robot
    handFrameName = robot.armChainNames[hand]
    trans = cart[handFrameName]
    if zFrom:
        diff = trans.inverse().compose(zFrom.cartConf()[handFrameName])
        dz = diff.matrix[2,3]
        print 'trans\n', trans.matrix
        print 'zFrom\n', zFrom.cartConf()[handFrameName].matrix
        print 'dz', dz
    if maxTarget:
        diff = trans.inverse().compose(maxTarget.cartConf()[handFrameName])
        max_dx = diff.matrix[0,3]
        print 'displaceHand', 'dx', dx, 'max_dx', max_dx
        dx = max(0., min(dx, max_dx)) # don't go past maxTrans
    nTrans = trans.compose(hu.Pose(dx, dy, dz, 0.0))
    basePose = (nearTo or conf).basePose()
    robot = conf.robot
    for nConf in robot.inverseKinWristGen(nTrans, hand, (nearTo or conf),
                                                basePose=basePose):
        if nConf['pr2Head'] is None:
            # Catch an apparent asymmetry in forward/inverse kin.
            nConf.conf['pr2Head'] = conf.conf['pr2Head']
        nConf.prettyPrint('displaceHand Conf:')
        return nConf
    print 'displaceHand: failed kinematics'
    return conf
    
        
