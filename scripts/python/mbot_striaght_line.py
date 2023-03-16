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

command.trans_v = 0.1
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

command.trans_v = 0
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(5.0)

command.trans_v = 0.2
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

command.trans_v = 0
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(1.0)

command.trans_v = 0.25
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

command.trans_v = 0
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(5.0)

command.trans_v = 0.3
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

command.trans_v = 0
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(1.0)
