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
import atexit
import math
import pydrake
import numpy as np
import subprocess
import signal
# TODO: get rid of unused imports
from robot_bhpn_drake_connection import RobotBhpnDrakeConnection

class Pr2JointsForBaseMovementBhpnDrakeConnection(RobotBhpnDrakeConnection):

    def __init__(self, bhpn_robot_conf):
        RobotBhpnDrakeConnection.__init__(self, bhpn_robot_conf)
        self.robot_type = 'pr2'
        self.num_joints = 24
        self.original_move_threshold = np.array([.015, .015, .015, .012, .012, .012, .012, .012, .012, .012, .012, .012, .012, .015, .015, .012, .012, .012, .012, .012, .012, .012, .015, .015])
        self.move_threshold = self.original_move_threshold.copy()
        self.move_threshold *= 4

    def get_bhpn_robot_conf(self, drake_robot_conf):
        drake_joints = drake_robot_conf.joint_position
        mapping = {'pr2Base': drake_joints[0:3],
                   'pr2Torso': drake_joints[3:4],
                   'pr2Head': drake_joints[4:6],
                   'pr2RightArm': drake_joints[6:13],
                   'pr2RightGripper': [np.sqrt(drake_joints[13:14][0]/100.0)],
                   'pr2LeftArm': drake_joints[15:22],
                   'pr2LeftGripper': [np.sqrt(drake_joints[22:23][0]/100.0)]}
        for k, v in mapping.items():
            self.bhpn_robot_conf = self.bhpn_robot_conf.set(k, list(v))
        return self.bhpn_robot_conf.copy()

    def get_drake_robot_conf(self, bhpn_robot_conf):
        msg = lcmt_robot_state()
        msg.timestamp = time.time() * 1000000
        msg.num_joints = self.num_joints
        msg.joint_position = self.get_joint_list(bhpn_robot_conf)
        msg.joint_robot = [0] * msg.num_joints
        msg.joint_name = [''] * msg.num_joints
        msg.joint_velocity = [0.0] * msg.num_joints
        print 'msg: ', msg
        return msg
 
    def get_joint_list(self, bhpn_robot_conf):
        return bhpn_robot_conf['pr2Base']\
            + bhpn_robot_conf['pr2Torso']\
            + bhpn_robot_conf['pr2Head']\
            + bhpn_robot_conf['pr2RightArm']\
            + [min([0.5, max([0.15, 100*bhpn_robot_conf['pr2RightGripper'][0]**2])])] * 2\
            + bhpn_robot_conf['pr2LeftArm']\
            + [min([0.5, max([0.15, 100*bhpn_robot_conf['pr2LeftGripper'][0]**2])])] * 2

    def get_joint_list_names(self):
        return ['x', 'y', 'theta', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint']
    
    def get_num_joints(self):
        return self.num_joints

    def get_move_threshold(self):
        return self.move_threshold.copy()

    def set_ignored_move_threshold(self, index, ignore):
        if ignore:
            self.move_threshold[index] = float('inf')
        else:
            self.move_threshold[index] = self.original_move_threshold[index]

    def get_hands_to_end_effectors(self):
        return {'left':'l_gripper_palm_link', 'right':'r_gripper_palm_link'}

    def get_drake_hand_joint_indices(self):
        return {'left': (22, 23), 'right': (13, 14)}

    def get_drake_continuous_joint_indices(self):
        return (2, 8, 10, 12, 17, 19, 21)

    def drake_soda_gripped_function(self, object_name, bhpn_drake_interface_obj):
        contact_force_magnitude_threshold = 2.0
        contact_results = bhpn_drake_interface_obj.contact_results
        gripper_end_effector_to_gripped_objects = {'l_gripper_palm_link':[], 'r_gripper_palm_link':[]}
        necessary_collisions_for_l_gripper_grip = set(['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
        necessary_collisions_for_r_gripper_grip = set(['r_gripper_l_finger_tip_link']) # TODO: ask why I have to do this , 'r_gripper_r_finger_tip_link'])
        necessary_collisions_combined = set()
        necessary_collisions_combined.update(necessary_collisions_for_l_gripper_grip)
        necessary_collisions_combined.update(necessary_collisions_for_r_gripper_grip)
        relevant_contact_results = filter(lambda contact_info: ((contact_info.body1_name == object_name and (contact_info.body2_name in necessary_collisions_combined)) or (contact_info.body2_name == object_name and (contact_info.body1_name in necessary_collisions_combined))) and np.linalg.norm(np.array(contact_info.contact_force)) >= contact_force_magnitude_threshold, contact_results.contact_info)
        relevant_contact_results_names = map(lambda contact_info: contact_info.body1_name if contact_info.body2_name == object_name else contact_info.body2_name, relevant_contact_results)
        relevant_names = set(relevant_contact_results_names)
        if necessary_collisions_for_l_gripper_grip.issubset(relevant_names):
            gripper_end_effector_to_gripped_objects['l_gripper_palm_link'] += [object_name]
        if necessary_collisions_for_r_gripper_grip.issubset(relevant_names):
            gripper_end_effector_to_gripped_objects['r_gripper_palm_link'] += [object_name]
        return gripper_end_effector_to_gripped_objects

    def get_object_gripped_function(self, object_type):
        if object_type == 'drake_soda':
            return self.drake_soda_gripped_function
        else:
            raise ValueError('A method to tell if this object has been gripped has not been provided')
    
    def get_gripped_objects(self, objects_to_check, bhpn_drake_interface_obj):
        gripped_results = {}
        for object_name, object_type in objects_to_check:
            gripped_results[object_name] = self.get_object_gripped_function(object_type)(object_name, bhpn_drake_interface_obj)
        return gripped_results

    def maintain_pick_conf(self, bhpn_robot_conf, hand):
        min_width_closed = 0.01
        return bhpn_robot_conf.set(bhpn_robot_conf.robot.gripperChainNames[hand], [min_width_closed])

    def pick(self, start_conf, target_conf, hand, obj, bhpn_drake_interface_obj):
        # Basic picking procedure. Not really that reactive (except it does ensure that the object is gripped hard enough). Can easily be made more reactive by taking more advantage of bhpn_drake_interface_obj's data from drake.
        
        # Constants
        step_time = 100000
        dx = 0.07
        dy = 0.05
        dz = 0.0
        width_open = 0.07
        min_width_closed = 0.01

        # Move gripper around object.
        start_conf_open = start_conf.set(start_conf.robot.gripperChainNames[hand], [width_open])
        target_conf_open= target_conf.set(target_conf.robot.gripperChainNames[hand], [width_open])
        n_conf = displaceHand(target_conf_open, hand, dx, dy, dz, nearTo=target_conf_open)
        
        # Close gripper        
        n_conf_closed = n_conf.set(n_conf.robot.gripperChainNames[hand], [min_width_closed])
               
        # Return to where the gripper started, holding the object (hopefully).
        start_conf_closed = start_conf.set(start_conf.robot.gripperChainNames[hand], n_conf_closed[n_conf_closed.robot.gripperChainNames[hand]])
        
        # Compute and do the path, given those waypoints
        path = rrt.interpolatePath([bhpn_drake_interface_obj.get_bhpn_robot_conf(), start_conf_open, target_conf_open, n_conf, n_conf_closed, start_conf_closed], 0.01)
        plan = bhpn_drake_interface_obj.encode_drake_robot_plan(path, [step*step_time for step in range(len(path))])
        bhpn_drake_interface_obj.command_drake_robot_plan(plan)
        
        return bhpn_drake_interface_obj.is_gripped(hand, obj)

    def place(self, start_conf, target_conf, hand, obj, bhpn_drake_interface_obj, timeout=60):
        # Basic placing procedure. Not really that reactive (except it does ensure that the object is not gripped). Can easily be made more reactive by taking more advantage of bhpn_drake_interface_obj's data from drake.
        
        # Constants
        step_time = 100000
        width_open = 0.07
     
        # Move gripper to target conf and open it
        hand_closed_conf = bhpn_drake_interface_obj.get_bhpn_robot_conf()[bhpn_drake_interface_obj.get_bhpn_robot_conf().robot.gripperChainNames[hand]]
        start_conf_correct_hand_conf = start_conf.set(start_conf.robot.gripperChainNames[hand], hand_closed_conf)
        target_conf_correct_hand_conf = target_conf.set(target_conf.robot.gripperChainNames[hand], hand_closed_conf)
        path = rrt.interpolatePath([bhpn_drake_interface_obj.get_bhpn_robot_conf(), start_conf_correct_hand_conf, target_conf_correct_hand_conf], 0.01)
        target_conf_open = target_conf.set(target_conf.robot.gripperChainNames[hand], [width_open])
        path += [target_conf_open, target_conf_open, target_conf_open, target_conf_open]

        plan = bhpn_drake_interface_obj.encode_drake_robot_plan(path, [step*step_time for step in range(len(path))])
        bhpn_drake_interface_obj.command_drake_robot_plan(plan)
        
        return not bhpn_drake_interface_obj.is_gripped(hand, obj)


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
    for n_conf in robot.inverseKinWristGen(nTrans, hand, (nearTo or conf),
                                                basePose=basePose):
        if n_conf['pr2Head'] is None:
            # Catch an apparent asymmetry in forward/inverse kin.
            n_conf.conf['pr2Head'] = conf.conf['pr2Head']
        n_conf.prettyPrint('displaceHand Conf:')
        return n_conf
    print 'displaceHand: failed kinematics'
    return conf
