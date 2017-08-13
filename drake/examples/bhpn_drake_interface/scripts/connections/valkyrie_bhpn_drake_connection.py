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

class ValkyrieBhpnDrakeConnection(RobotBhpnDrakeConnection):
    # Note: since valkyrie has legs, but BHPN doesn't command legs (BHPN only
    # commands the planar x, y, theta position of the torso), the joint
    # configuration that is sent to drake contains the x, y, theta position of
    # the valkyrie's torso, followed by the joint values of the valkyrie's
    # upper body joints. So the robot_plan_t that will get sent to the
    # valkyrie drake controller will need to be further broken down. The
    # controller can follow the upper body joint values directly, but the x,
    # y, theta values will need to first be passed to a walking plannner in
    # drake to yield the correct lower body joint values for the controller.

    def __init__(self, bhpn_robot_conf):
        RobotBhpnDrakeConnection.__init__(self, bhpn_robot_conf)
        self.robot_type = 'valkyrie'
        self.num_drake_joints = 40

    def get_bhpn_robot_conf(self, drake_robot_conf):
        drake_joints = drake_robot_conf.joint_position
        mapping = {'robotBase': drake_joints[0:3],
                   'robotLeftArm': drake_joints[3:10],
                   'robotRightArm': drake_joints[10:17],
                   'robotHead': drake_joints[17:18],
                   'robotLeftGripper': drake_joints[18:20],
                   'robotRightGripper': drake_joints[20:22]}
        for k, v in mapping.items():
            self.bhpn_robot_conf = self.bhpn_robot_conf.set(k, list(v))
        return self.bhpn_robot_conf.copy()

    def interpolate_drake_robot_confs(self, bhpn_robot_confs, time_spacing=100000):

        drake_robot_confs = []
        utime = time_spacing
        for bhpn_robot_conf in bhpn_robot_confs:
            state = robot_state_t()
            state.utime = utime
            state.num_joints = self.num_drake_joints
            state.joint_name = self.get_joint_list_names()
            state.joint_position = [0]*9 + self.get_joint_list_only_bhpn_joints(bhpn_robot_conf)[:15] + [0, 0, -0.49, 1.205, -0.71, 0, 0, 0, -0.49, 1.205, -0.71, 0] + get_joint_list_only_bhpn_joints(bhpn_robot_conf)[15:]
            state.joint_velocity = [0]*state.num_drake_joints
            state.joint_effort = [0]*state.num_drake_joints
            drake_robot_confs.append(state)
            utime += time_spacing
        return drake_robot_confs

    def get_joint_list_only_bhpn_joints(self, bhpn_robot_conf):
        return bhpn_robot_conf['robotHead']\
            + bhpn_robot_conf['robotRightArm']\
            + bhpn_robot_conf['robotLeftArm']\
            + bhpn_robot_conf['robotLeftGripper']\
            + bhpn_robot_conf['robotRightGripper']

    def get_joint_list_names(self):
        return [
            'x',
            'y',
            'z',
            'roll',
            'pitch',
            'yaw',
            'torsoYaw',
            'waistLeftActuator',
            'waistRightActuator',
            'lowerNeckPitch',
            'rightShoulderPitch',
            'rightShoulderRoll',
            'rightShoulderYaw',
            'rightElbowPitch',
            'rightForearmYaw',
            'rightWristTopActuator',
            'rightWristBottomActuator',
            'leftShoulderPitch',
            'leftShoulderRoll',
            'leftShoulderYaw',
            'leftElbowPitch',
            'leftForearmYaw',
            'leftWristTopActuator',
            'leftWristBottomActuator',
            'rightHipYaw',
            'rightHipRoll',
            'rightHipPitch',
            'rightKneePitch',
            'rightAnkleInsideActuator',
            'rightAnkleOutsideActuator',
            'leftHipYaw',
            'leftHipRoll',
            'leftHipPitch',
            'leftKneePitch',
            'leftAnkleInsideActuator',
            'leftAnkleOutsideActuator',
            'leftThumbPitch1Actuator',
            'leftMiddleFingerPitch1Actuator',
            'rightThumbPitch1Actuator',
            'rightMiddleFingerPitch1Actutor']

    def get_hands_to_end_effectors(self):
        return {'left': 'leftPalm', 'right': 'rightPalm'}

    def get_drake_hand_joint_indices(self):
        return {'left': (22, 23), 'right': (13, 14)}

    def get_drake_continuous_joint_indices(self):
        return (2)

    def drake_soda_gripped_function(
            self, object_name, bhpn_drake_interface_obj):
        contact_force_magnitude_threshold = 0.005
        contact_results = bhpn_drake_interface_obj.contact_results
        gripper_end_effector_to_gripped_objects = {
            'leftPalm': [], 'rightPalm': []}
        necessary_collisions_for_l_gripper_grip = set(
            ['leftIndexFingerPitch3Link', 'leftThumbPitch3Link'])\
        necessary_collisions_for_r_gripper_grip = set(
            ['rightIndexFingerPitch3Link', 'rightThumbPitch3Link'])
        necessary_collisions_combined = set()
        necessary_collisions_combined.update(
            necessary_collisions_for_l_gripper_grip)
        necessary_collisions_combined.update(
            necessary_collisions_for_r_gripper_grip)
        relevant_contact_results = filter(
            lambda contact_info: (
                (contact_info.body1_name == object_name and (
                    contact_info.body2_name in necessary_collisions_combined)) or (
                    contact_info.body2_name == object_name and (
                        contact_info.body1_name in necessary_collisions_combined))) and np.linalg.norm(
                np.array(
                    contact_info.contact_force)) >= contact_force_magnitude_threshold,
            contact_results.contact_info)
        relevant_contact_results_names = map(
            lambda contact_info: contact_info.body1_name if contact_info.body2_name == object_name else contact_info.body2_name,
            relevant_contact_results)
        relevant_names = set(relevant_contact_results_names)
        if necessary_collisions_for_l_gripper_grip.issubset(relevant_names):
            gripper_end_effector_to_gripped_objects['leftPalm'] += [
                object_name]
        if necessary_collisions_for_r_gripper_grip.issubset(relevant_names):
            gripper_end_effector_to_gripped_objects['rightPalm'] += [
                object_name]
        return gripper_end_effector_to_gripped_objects

    def get_object_gripped_function(self, object_type):
        if object_type == 'drake_soda':
            return self.drake_soda_gripped_function
        else:
            raise ValueError(
                'A method to tell if this object has been gripped has not been provided')

    def get_gripped_objects(self, objects_to_check, bhpn_drake_interface_obj):
        gripped_results = {}
        for object_name, object_type in objects_to_check:
            gripped_results[object_name] = self.get_object_gripped_function(
                object_type)(object_name, bhpn_drake_interface_obj)
        return gripped_results

    def maintain_pick_conf(self, bhpn_robot_conf, hand):
        # TODO: redo this for valkyrie.
        return bhpn_robot_conf.set(
            start_conf.robot.gripperChainNames[hand],
            [min_width_closed])

    def pick(
            self,
            start_conf,
            target_conf,
            hand,
            obj,
            bhpn_drake_interface_obj):
        # TODO: redo this for valkyrie.
        # Basic picking procedure. Not really that reactive (except it does
        # ensure that the object is gripped hard enough). Can easily be made
        # more reactive by taking more advantage of bhpn_drake_interface_obj's
        # data from drake.

        # Constants
        step_time = 100000
        dx = 0.07
        dy = 0.05
        dz = 0.0
        width_open = 0.07
        min_width_closed = 0.035

        # Move gripper around object.
        start_conf_open = start_conf.set(
            start_conf.robot.gripperChainNames[hand], [width_open])
        target_conf_open = target_conf.set(
            target_conf.robot.gripperChainNames[hand], [width_open])
        n_conf = displaceHand(
            target_conf_open,
            hand,
            dx,
            dy,
            dz,
            nearTo=target_conf_open)

        # Close gripper
        n_conf_closed = n_conf.set(
            n_conf.robot.gripperChainNames[hand],
            [min_width_closed])

        # Return to where the gripper started, holding the object (hopefully).
        start_conf_closed = start_conf.set(
            start_conf.robot.gripperChainNames[hand], n_conf_closed[n_conf_closed.robot.gripperChainNames[hand]])

        # Compute and do the path, given those waypoints
        path = rrt.interpolatePath([bhpn_drake_interface_obj.get_bhpn_robot_conf(
        ), start_conf_open, target_conf_open, n_conf, n_conf_closed, start_conf_closed], 0.01)
        plan = bhpn_drake_interface_obj.encode_drake_robot_plan(
            path, [step * step_time for step in range(len(path))])
        bhpn_drake_interface_obj.command_drake_robot_plan(plan)

        return bhpn_drake_interface_obj.is_gripped(hand, obj)

    def place(
            self,
            start_conf,
            target_conf,
            hand,
            obj,
            bhpn_drake_interface_obj,
            timeout=60):
        # Basic placing procedure. Not really that reactive (except it does
        # ensure that the object is not gripped). Can easily be made more
        # reactive by taking more advantage of bhpn_drake_interface_obj's data
        # from drake.
        # TODO: redo this for valkyrie.

        # Constants
        step_time = 100000
        width_open = 0.07

        # Move gripper to target conf and open it
        hand_closed_conf = bhpn_drake_interface_obj.get_bhpn_robot_conf()[
            bhpn_drake_interface_obj.get_bhpn_robot_conf().robot.gripperChainNames[hand]]
        start_conf_correct_hand_conf = start_conf.set(
            start_conf.robot.gripperChainNames[hand], hand_closed_conf)
        target_conf_correct_hand_conf = target_conf.set(
            target_conf.robot.gripperChainNames[hand], hand_closed_conf)
        path = rrt.interpolatePath([bhpn_drake_interface_obj.get_bhpn_robot_conf(
        ), start_conf_correct_hand_conf, target_conf_correct_hand_conf], 0.01)
        target_conf_open = target_conf.set(
            target_conf.robot.gripperChainNames[hand], [width_open])
        path += [target_conf_open, target_conf_open,
                 target_conf_open, target_conf_open]

        plan = bhpn_drake_interface_obj.encode_drake_robot_plan(
            path, [step * step_time for step in range(len(path))])
        bhpn_drake_interface_obj.command_drake_robot_plan(plan)

        return not bhpn_drake_interface_obj.is_gripped(hand, obj)


def displaceHand(conf, hand, dx=0.0, dy=0.0, dz=0.0,
                 zFrom=None, maxTarget=None, nearTo=None):
    # TODO: redo this for valkyrie.
    print 'displaceHand'
    cart = conf.cartConf()
    robot = conf.robot
    handFrameName = robot.armChainNames[hand]
    trans = cart[handFrameName]
    if zFrom:
        diff = trans.inverse().compose(zFrom.cartConf()[handFrameName])
        dz = diff.matrix[2, 3]
        print 'trans\n', trans.matrix
        print 'zFrom\n', zFrom.cartConf()[handFrameName].matrix
        print 'dz', dz
    if maxTarget:
        diff = trans.inverse().compose(maxTarget.cartConf()[handFrameName])
        max_dx = diff.matrix[0, 3]
        print 'displaceHand', 'dx', dx, 'max_dx', max_dx
        dx = max(0., min(dx, max_dx))  # don't go past maxTrans
    nTrans = trans.compose(hu.Pose(dx, dy, dz, 0.0))
    basePose = (nearTo or conf).basePose()
    robot = conf.robot
    for n_conf in robot.inverseKinWristGen(nTrans, hand, (nearTo or conf),
                                           basePose=basePose):
        if n_conf['robotHead'] is None:
            # Catch an apparent asymmetry in forward/inverse kin.
            n_conf.conf['robotHead'] = conf.conf['robotHead']
        n_conf.prettyPrint('displaceHand Conf:')
        return n_conf
    print 'displaceHand: failed kinematics'
    return conf
