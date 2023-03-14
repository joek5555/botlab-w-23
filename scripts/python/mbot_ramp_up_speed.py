import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t

fwd_vel = 0.25 # 0.25 m/s
turn_vel = 3.14 # pi rad/sec

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
time.sleep(0.5)
command = mbot_motor_command_t()


for i in range (100):
    command.trans_v = 0.01 * i
    command.angular_v = 0
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    print( command.trans_v)
    time.sleep(0.5)

for i in range (100):
    command.trans_v = 1 - 0.01 * i
    command.angular_v = 0
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    print( command.trans_v)
    time.sleep(0.5)