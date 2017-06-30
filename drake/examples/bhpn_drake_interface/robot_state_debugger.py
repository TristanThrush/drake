import lcm
from lcmt_robot_state import lcmt_robot_state

def robotConfCallback(channel, data):
    print lcmt_robot_state.decode(data).joint_position

lc = lcm.LCM()
lc.subscribe('DRAKE_ROBOT_STATE', robotConfCallback)

while True:
    lc.handle()         
