import os
import multiprocessing
import subprocess
import threading
from geometry import shapes, hu
from robot import conf
import lcm
from drake import lcmt_viewer_draw, lcmt_contact_results_for_viz
from bot_core import robot_state_t
from robotlocomotion import robot_plan_t, plan_status_t
from operator import sub
import time
import atexit
import math
import numpy as np
import subprocess
import signal
from pr2_joints_for_base_movement_bhpn_drake_connection import Pr2JointsForBaseMovementBhpnDrakeConnection
from valkyrie_bhpn_drake_connection import ValkyrieBhpnDrakeConnection

drake_path = '/Users/tristanthrush/research/mit/drake/'
interface_path = 'drake/examples/bhpn_drake_interface/'
interface_build_path = 'bazel-bin/drake/examples/bhpn_drake_interface/'
interface_path_absolute = drake_path + interface_path
interface_build_path_absolute = drake_path + interface_build_path

supported_object_types = {'drake_table': interface_path + 'objects/drake_table.sdf', 'drake_soda': interface_path + 'objects/drake_soda.urdf'}
supported_robot_types = {'pr2': (Pr2JointsForBaseMovementBhpnDrakeConnection, 'pr2_simulator', 'PR2_PLAN', 'ROBOT_STATE'), 'valkyrie': (ValkyrieBhpnDrakeConnection, 'valkyrie_simulator', 'VALKYRIE_MANIP_PLAN', 'EST_ROBOT_STATE')}

fun_colors = {'drake_soda1': 'drake_soda_salmon', 'drake_soda2': 'drake_soda_blue'}

class BhpnDrakeInterface:

    def __init__(
            self,
            robot_type,
            bhpn_robot_conf,
            object_types,
            bhpn_object_confs,
            fixed_objects,
            no_plan_status=False
            ):

        # Make sure that the drake simulation can actually be created with the given arguments.
        if not set([robot_type]).issubset(set(supported_robot_types.keys())):
            raise ValueError('This robot is unsupported by the BHPN-Drake Interface.')
        if not set(object_types.values()).issubset(set(supported_object_types.keys())):
            raise ValueError('Some of these objects are unsupported by the BHPN-Drake Interface.')

        # Store the arguments.
        self.robot_type = robot_type
        self.bhpn_robot_conf = bhpn_robot_conf.copy()
        self.object_types = object_types.copy()
        self.bhpn_object_confs = bhpn_object_confs.copy()
        self.fixed_objects = fixed_objects.copy()

        # Create the robot connection and get the simulator that is specific to the type of robot.
        self.robot_connection = supported_robot_types[self.robot_type][0](bhpn_robot_conf)       
        self.robot_simulator_executable = supported_robot_types[self.robot_type][1]
        
        # Information about the drake simulation that will be properly initialized before this class is initialized.
        self.drake_robot_conf = None
        self.contact_results = None
        self.plan_status = None
        self.no_plan_status = no_plan_status
        if self.no_plan_status:
            self.plan_status = plan_status_t()
        self.generated_description_paths = []

        # Start the lcm subscribers that listen to the drake simulatiuon for information.
        self.lc = lcm.LCM()
        self.lc.subscribe(supported_robot_types[self.robot_type][3], self.robot_conf_callback)
        self.drake_viewer_draw_subscription = self.lc.subscribe('DRAKE_VIEWER_DRAW', self.object_pose_callback)
        self.contact_results_subscription = self.lc.subscribe('CONTACT_RESULTS', self.contact_results_callback)
        self.plan_status_subscription = self.lc.subscribe('PLAN_STATUS', self.plan_status_callback)
        self.callback_handler = threading.Thread(target=self.handle_lcm_subscribers)
        self.callback_handler.daemon = True
        self.callback_handler.start()
        
        # Start the drake simulation.
        self.create_bdisc()
        self.drake_simulation = subprocess.Popen('cd ' + drake_path + '; exec ' + interface_build_path + self.robot_simulator_executable + ' ' + interface_path + 'tmp/simulator_conf.bdisc', shell=True)
        
        # Wait to get full information from drake simulation.
        while self.drake_robot_conf is None or self.contact_results is None or self.plan_status is None:
            time.sleep(0.1)

        atexit.register(self.release)
        print 'Initialized the BHPN-Drake Interface.'

    ############# Initialization/Destruction methods ##########################

    def release(self):
        print 'Attempting to terminate the BHPN-Drake Interface.'
        self.drake_simulation.terminate()
        return_code = self.drake_simulation.wait()
        print 'Terminated the Drake simulation. Return code of the Drake simulation: ', return_code
        for path in self.generated_description_paths:
            os.system('rm ' + path)
        os.system('rm ' + interface_path_absolute + 'tmp/simulator_conf.bdisc')
        print "Removed temporary objects."
        print 'Terminated the BHPN-Drake Interface.'

    def generate_description_for_object_name(self, object_name, object_type):
        fun_color_replacement_type = object_type
        if object_name in fun_colors:
            fun_color_replacement_tyIpe = fun_colors[object_name]
        type_description_path = drake_path + supported_object_types[object_type].replace(object_type, fun_color_replacement_type)
        type_description = open(type_description_path, 'r')
        text = type_description.read()
        type_description.close()
        text = text.replace(object_type, object_name)
        # I know that the following is not robost to every path name, but I think it will be fine (hopefully)
        name_description_path = type_description_path.replace(object_type, object_name).replace('objects', 'tmp') 
        name_description = open(name_description_path, 'w')
        name_description.write(text)
        name_description.close()
        self.generated_description_paths.append(name_description_path)
        return supported_object_types[object_type].replace(object_type, fun_color_replacement_type).replace(object_type, object_name).replace('objects', 'tmp')

    def create_bdisc(self):
        self.fixed_objects = ['drake_table', 'drake_table1', 'drake_table2']
        initial_robot_pose = '0 0 0 0 0 0'
        initial_robot_joint_positions = ''
        for joint in self.robot_connection.interpolate_drake_robot_confs([self.bhpn_robot_conf])[0].joint_position:
            initial_robot_joint_positions += str(joint) + ' '
        objects = []
        for object_name, object_type in self.object_types.items():
            object_info = ''
            object_info += object_name + ' '
            if object_name != object_type:
                object_info += self.generate_description_for_object_name(object_name, object_type) + ' '
            else:
                object_info += supported_object_types[object_type] + ' '
            for value in self.convert_to_drake_pose(self.bhpn_object_confs[object_name][object_name][0]):
                object_info += str(value) + ' '
            if object_name in self.fixed_objects:
                object_info += 'true '
            else:
                object_info += 'false '
            objects.append(object_info)
        simulator_conf_text = initial_robot_pose + '\n'
        simulator_conf_text += initial_robot_joint_positions + '\n'
        num_objects = len(objects)
        for index in range(num_objects):
            if index != num_objects - 1:
                simulator_conf_text += objects[index] + '\n'
            else:
                simulator_conf_text += objects[index]
        simulator_conf_file = open(interface_path_absolute + 'tmp/simulator_conf.bdisc', 'w')
        simulator_conf_file.write(simulator_conf_text)
        simulator_conf_file.close()
        print 'Created simulator_conf_file:'
        print simulator_conf_text

    ######### Simulation command methods ######################################

    def encode_drake_robot_plan(self, bhpn_robot_confs, time_spacing):
        drake_confs = self.robot_connection.interpolate_drake_robot_confs(bhpn_robot_confs, time_spacing)
        plan = robot_plan_t()
        plan.utime = 0
        plan.robot_name = self.robot_type 
        plan.num_states = len(drake_confs)
        plan.plan_info = [1]*plan.num_states 
        last_joint_values_on_path = self.drake_robot_conf.joint_position 
        for state in drake_confs:
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

        last_plan_start_utime = self.plan_status.last_plan_start_utime
        print last_plan_start_utime
        self.lc.publish(supported_robot_types[self.robot_type][2], plan.encode())

        if not self.no_plan_status:
            # TODO: fix this!
            # Wait for plan to be recieved.
            print "Waiting for plan to be recieved."
            counter = 0
            while self.plan_status.last_plan_start_utime == last_plan_start_utime and counter < 50:
                counter += 1
                time.sleep(0.1)
            print self.plan_status.last_plan_start_utime
            print "Plan recieved."
        
            print "Waiting for plan to finish."
            # Wait for plan to finish.
            while self.plan_status.execution_status == 0:
                time.sleep(0.1)
            print "Plan finished."
    
        print "Done following Drake robot plan."

    ######### Getter methods ##################################################

    def get_bhpn_robot_conf(self):
        return self.bhpn_robot_conf.copy()

    def get_bhpn_object_confs(self):
        return self.bhpn_object_confs.copy()

    def is_gripped(self, hand, object_name):
        return object_name in self.robot_connection.get_gripped_objects([(object_name, self.object_types[object_name])], self)[object_name][self.robot_connection.get_hands_to_end_effectors()[hand]]

    # BHPN Primitives that are robot-secific, and supported in the interface ##

    def pick(self, start_conf, target_conf, hand, obj):
        return self.robot_connection.pick(start_conf, target_conf, hand, obj, self)

    def maintain_pick_conf(self, bhpn_robot_conf, hand):
        return self.robot_connection.maintain_pick_conf(bhpn_robot_conf, hand)

    def place(self, start_conf, target_conf, hand, obj):
        return self.robot_connection.place(start_conf, target_conf, hand, obj, self)

    ######### LCM methods #####################################################

    def handle_lcm_subscribers(self):
        while True:
            self.lc.handle()

    def robot_conf_callback(self, channel, data):
        self.drake_robot_conf = robot_state_t.decode(data)
        self.bhpn_robot_conf = self.robot_connection.get_bhpn_robot_conf(self.drake_robot_conf)
     
    def object_pose_callback(self, channel, data):
        msg = lcmt_viewer_draw.decode(data)
        for obj in self.bhpn_object_confs:
             self.bhpn_object_confs[obj] = self.convert_to_bhpn_pose(msg.position[msg.link_name.index(obj)], msg.quaternion[msg.link_name.index(obj)])
                
    def contact_results_callback(self, channel, data):
        self.contact_results = lcmt_contact_results_for_viz.decode(data)
    
    def plan_status_callback(self, channel, data):
        self.plan_status = plan_status_t.decode(data)
             
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
