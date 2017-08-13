# This example simply loads up the BHPN-Drake interface with the pr2 and calls
# on the move primitive action to execute an interesting hard coded path.
# This example does not run BHPN, just the interface. The code in this example
# simply executes a primitive that BHPN could execute with the interface.
# Also, be sure to start drake_visualizer to see what's going on!

import bhpn_drake_interface

robot_type = 'pr2'
bhpn_robot_conf = 

interface = bhpn_drake_interface.BhpnDrakeInterface(


