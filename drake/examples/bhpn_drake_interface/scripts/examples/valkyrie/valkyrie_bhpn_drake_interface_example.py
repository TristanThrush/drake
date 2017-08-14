# on the move primitive action to execute an interesting hard coded path.
# This example does not run BHPN, just the interface. The code in this example
# simply executes a primitive that BHPN could execute with the interface.
# Also, be sure to start drake_visualizer to see what's going on!

import bhpn_drake_interface
from geometry.hu import Pose
from create_general_robot import makeRobot
import numpy as np
import time

robot_type = 'valkyrie'
bhpn_robot_conf = makeRobot(np.array([(-1.0, -3.0, 0.0), (3.0, 3.0, 2.0)])).nominalConf
bhpn_robot_conf = bhpn_robot_conf.set('robotRightArm', [0.3, 1.25, 0, 0.9, 1.571, 0, 0])
bhpn_robot_conf = bhpn_robot_conf.set('robotLeftArm', [0.3, -1.25, 0, -0.785, 1.571, 0, 0])
bhpn_robot_conf = bhpn_robot_conf.set('robotLeftGripper', [0])
bhpn_robot_conf = bhpn_robot_conf.set('robotRightGripper', [0])
object_types = {'drake_soda': 'drake_soda', 'drake_table': 'drake_table'}
soda_pose = Pose(1.63, -0.37, 0.83, 1.17)
table_pose = Pose(1.97, -0.03, 0.38225, 0)
bhpn_object_confs = {'drake_soda': {'drake_soda': [soda_pose]}, 'drake_table': {'drake_table': [table_pose]}}
fixed_objects = {'drake_table'}

interface = bhpn_drake_interface.BhpnDrakeInterface(robot_type, bhpn_robot_conf, object_types, bhpn_object_confs, fixed_objects, True)

print 'Sending example plan'

# Make an example BHPN motion plan that picks and places an object.
bhpn_motion_plan = []
for index in range(6):
    bhpn_robot_conf = bhpn_robot_conf.set('robotRightArm', [0.3 - index*0.095, 1.25, 0, 0.9, 1.571, 0, 0])
    bhpn_motion_plan.append(bhpn_robot_conf)
for index in range(15):
    bhpn_robot_conf = bhpn_robot_conf.set('robotRightGripper', [-index*0.1])
    bhpn_motion_plan.append(bhpn_robot_conf)
for index in range(8):
    bhpn_robot_conf = bhpn_robot_conf.set('robotRightArm', [-0.2 - index*0.1, 1.25, 0, 0.9, 1.571, 0, 0])
    bhpn_motion_plan.append(bhpn_robot_conf)

# Tell the interface to execute this primitive motion plan.
drake_motion_plan = interface.encode_drake_robot_plan(bhpn_motion_plan, 100000)
for conf in drake_motion_plan.plan:
    print conf.joint_position
    print conf.joint_velocity
interface.command_drake_robot_plan(drake_motion_plan)

print "letting simulation run for 1000 seconds"
time.sleep(1000)
