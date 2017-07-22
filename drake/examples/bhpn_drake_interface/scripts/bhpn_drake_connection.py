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

supported_object_types = {'drake_table': interface_path + 'objects/drake_table.sdf', 'drake_soda': interface_path + 'objects/drake_soda.urdf'}
supported_robot_types = {'pr2': Pr2JointsForBaseMovementBhpnDrakeConnection}

class BhpnDrakeInterface:

    def __init__(
            self,
            robot_type,
            bhpn_robot_conf,
            fixed_robot,
            object_types,
            bhpn_object_confs,
            fixed_objects,
            ):

        # Make sure that the drake simulation can actually be created with the given arguments.
        if not set(robot_type).issubset(set(supported_robot_types.keys())):
            raise ValueError('This robot is unsupported by the BHPN-Drake Interface.')
        if not set(object_types.values()).issubset(set(supported_object_types.keys())):
            raise ValueError('Some of these objects are unsupported by the BHPN-Drake Interface.')
        if bhpn_robot_conf.getBasePose() != [0., 0., 0.]:
            raise ValueError('The BHPN-Drake Interface requires that the base of the robot be at 0, 0, 0.')

        # Store the arguments.
        self.robot_type = robot_type
        self.bhpn_robot_conf = bhpn_robot_conf.copy()
        self.fixed_robot = fixed_robot
        self.object_types = object_types.copy()
        self.bhpn_object_confs = bhpn_object_confs.copy()
        self.fixed_objects = fixed_objects.copy()

        # Create the robot connection (implementation of primitives, etc.) that is specific to the type of robot.
        self.robot_connection = supported_robots[self.robot_type](bhpn_robot_conf)        
        
        # Information about the drake simulation that will be properly initialized before this class is initialized.
        self.drake_robot_conf = None
        self.contact_results = None

        # Start the lcm subscribers that listen to the drake simulatiuon for information.
        self.lc = lcm.LCM()
        self.lc.subscribe('ROBOT_STATE', self.robot_conf_callback)
        self.lc.subscribe('DRAKE_VIEWER_DRAW', self.object_pose_callback)
        self.lc.subscribe('CONTACT_RESULTS', self.contact_results_callback)
        self.handler_poison_pill = False
        self.robot_conf_handler = threading.Thread(target=self.handle_lcm_subscribers)
        self.robot_conf_handler.daemon = True
        self.robot_conf_handler.start()
        
        # Start the drake simulation.
        self.drake_simulation = subprocess.Popen('cd ' + interface_build_path_absolute + '; ' + 'exec ' + self.create_drake_simulation_command(), shell=True)
        
        # Wait to get full information from drake simulation.
        while self.drake_robot_conf is None or self.contact_results is None:
            time.sleep(0.1)

        atexit.register(self.release)
        print 'Initialized the BHPN-Drake Interface.'

    ############# Initialization/Destruction methods ##########################

    def release(self):
        print 'Attempting to terminate the BHPN-Drake Interface.'
        self.handler_poison_pill = True
        self.robot_conf_handler.join()
        self.drake_simulation.terminate()
        return_code = self.drake_simulation.wait()
        print 'Return code of the Drake simulation: ', return_code
        print 'Terminated the BHPN-Drake Interface.'

    def create_drake_simulation_command(self):
        command = interface_build_path_absolute + 'simulation '
        command += str(self.robot_connection.get_num_joints()) + ' '
        for joint in self.robot_connection.get_drake_robot_conf(self.bhpn_robot_conf).joint_position:
            command += str(joint) + ' '
        command += self.robot_type + ' '
        command += self.robot_connection.get_urdf_path() + ' '
        command += '0 0 0 0 0 0 '
        command += str(self.fixed_robot).lower() + ' '
        for name, object_type in self.object_types.items():
            command += name + ' '
            command += supported_object_types[object_type] + ' '
            for value in self.convert_to_drake_pose(self.bhpn_object_confs[name][name][0]):
                    command += str(value) + ' '
            if name in self.fixed_objects:
                command += 'true '
            else:
                command += 'false '
        print 'Created Drake simulation command.'
        print command
        return command

    ######### Simulation command methods ######################################

    def encode_drake_robot_plan(self, bhpn_robot_path, time):
        assert len(time) == len(bhpn_robot_path)
        plan = robot_plan_t()
        plan.utime = 0
        plan.robot_type = self.robot_type 
        plan.num_states = len(time)
        plan.plan_info = [1]*plan.num_states 
        last_joint_values_on_path = self.drake_robot_conf.joint_position 
        for index in range(len(bhpn_robot_path)):
            state = robot_state_t()
            state.utime = time[index]
            state.num_joints = self.robot_connection.get_num_joints()
            state.joint_name = self.robot_connection.get_joint_list_names()
            state.joint_position = self.robot_connection.get_joint_list(bhpn_robot_path[index])
            state.joint_velocity = [0]*state.num_joints
            state.joint_effort = [0]*state.num_joints
            # Modify the plan so that drake wont stupidly take the long way from 0 to 2 pi for continuous joints.
            for continuous_joint_index in self.robot_connection.get_drake_continuous_joint_indices():
                if state.joint_position[continuous_joint_index] - last_joint_values_on_path[continuous_joint_index] > math.pi:
                    state.joint_position[continuous_joint_index] -= 2*math.pi
                if state.joint_position[continuous_joint_index] - last_joint_values_on_path[continuous_joint_index] < -1*math.pi:
                    state.joint_position[continuous_joint_index] += 2*math.pi
                assert abs(state.joint_position[continuous_joint_index] - last_joint_values_on_path[continuous_joint_index]) <= math.pi + 0.01
            last_joint_values_on_path = state.joint_position
            plan.plan.append(state)
        plan.num_grasp_transitions = 0
        plan.left_arm_control_type = plan.POSITION
        plan.right_arm_control_type = plan.POSITION
        plan.left_leg_control_type = plan.POSITION
        plan.right_leg_control_type = plan.POSITION
        plan.num_bytes = 0
        return plan

    def command_drake_robot_plan(self, plan):
        print "Attempting to follow Drake robot plan."
        self.lc.publish('ROBOT_PLAN', plan.encode())
        # TODO: think about the best way to consider this plan finished instead of the below while loop.
        while np.max(np.abs(np.array(plan.plan[-1].joint_position) - np.array(self.drake_robot_conf.joint_position)) - self.robot_connection.get_move_threshold()) > 0:
            
            # If an object is gripped relax threshold for the gripper.
            gripped = self.robot_connection.get_gripped_objects(self.contact_results, self.get_bhpn_object_confs().keys())
            for hand, endEffector in self.robot_connection.get_hands_to_end_effectors().items():
                print gripped[endEffector]
                if gripped[endEffector] != []:
                    for index in self.robot_connection.get_drake_hand_joint_indices()[hand]:
                        self.robot_connection.set_ignored_move_threshold(index, True)
                else:
                    for index in self.robot_connection.get_drake_hand_joint_indices()[hand]:
                        self.robot_connection.set_ignored_move_threshold(index, False)
            
            print np.abs(np.array(plan.plan[-1].joint_position) - np.array(self.drake_robot_conf.joint_position)) - self.robot_connection.get_move_threshold()

            time.sleep(0.1)
        print "Done following Drake robot plan."

    ######### Getter methods ##################################################

    def get_bhpn_robot_conf(self):
        return self.bhpn_robot_conf.copy()

    def get_bhpn_object_confs(self):
        return self.bhpn_object_confs.copy()

    def is_gripped(self, hand, obj):
        return obj in self.robot_connection.get_gripped_objects(self.contact_results, [obj])[self.robot_connection.get_hands_to_end_effectors()[hand]]

    # BHPN Primitives that are robot-secific, and supported in the interface ##

    def pick(self, start_conf, target_conf, hand, obj):
        return self.robot_connection.pick(start_conf, target_conf, hand, obj, self)

    def place(self):
        # TODO: implement.
        raise NotImplementedError

    ######### LCM methods #####################################################

    def handle_lcm_subscribers(self):
        while not self.handler_poison_pill:
            self.lc.handle()

    def robot_conf_callback(self, channel, data):
        self.drake_robot_conf = lcmt_robot_state.decode(data)
        self.bhpn_robot_conf = self.robot_connection.get_bhpn_robot_conf(self.drake_robot_conf)
     
    def object_pose_callback(self, channel, data):
        msg = lcmt_viewer_draw.decode(data)
        for obj in self.bhpn_object_confs:
            objPose = self.convert_to_bhpn_pose(msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
            self.bhpn_object_confs[obj] = objPose     

    def contact_results_callback(self, channel, data):
        self.contact_results = lcmt_contact_results_for_viz.decode(data)            
        
    ######### Conversion utility methods ######################################

    def convert_to_bhpn_pose(self, position, orientation):
        return hu.Pose(
            position[0],
            position[1],
            position[2],
            self.convert_from_drake_q_to_rpy(orientation)[2])
            
    def convert_to_drake_pose(self, pose):
        return [pose.x, pose.y, pose.z, 0, 0, pose.theta]

    def convert_from_drake_q_to_rpy(self, quaternion):
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
