# This example simply loads up the BHPN-Drake interface with the pr2 and calls
# on the move primitive action to execute an interesting hard coded path.
# This example does not run BHPN, just the interface. The code in this example
# simply executes a primitive that BHPN could execute with the interface.
# Also, be sure to start drake_visualizer to see what's going on!

import bhpn_drake_interface
from geometry.hu import Pose
from robot.pr2.pr2Robot import makeRobot
import numpy as np
import time

robot_type = 'pr2'
bhpn_robot_conf = makeRobot(np.array([(-1.0, -3.0, 0.0), (3.0, 3.0, 2.0)])).nominalConf
object_types = {'drake_soda': 'drake_soda', 'drake_table': 'drake_table'}
soda_pose = Pose(1.63, -0.37, 0.83, 1.17)
table_pose = Pose(1.97, -0.03, 0.38225, 0)
bhpn_object_confs = {'drake_soda': {'drake_soda': [soda_pose]}, 'drake_table': {'drake_table': [table_pose]}}
fixed_objects = {'drake_table'}

interface = bhpn_drake_interface.BhpnDrakeInterface(robot_type, bhpn_robot_conf, object_types, bhpn_object_confs, fixed_objects)

print "letting simulation run for 100 seconds"
time.sleep(100)
