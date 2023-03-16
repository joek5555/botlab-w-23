import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t
from mbot_lcm_msgs import mbot_encoder_t
import numpy as np
import matplotlib.pyplot as plt

fwd_vel = 0.1 # 0.25 m/s
turn_vel = 0 # pi rad/sec
dt = 0.05

WHEEL_BASE = 0.15
WHEEL_RADIUS = 0.04
ENCODER_RESOLUTION = 20
GEAR_RATIO = 78.0

#left_target_velocity_arr = np.array([0])
#right_target_velocity_arr = np.array([0])
#left_velocity_arr = np.array([0])
#right_velocity_arr = np.array([0])

left_target_velocity_arr = []
right_target_velocity_arr = []
left_velocity_arr = []
right_velocity_arr = []

def my_handler(channel, data):
    #print("went inside")
    msg = mbot_encoder_t.decode(data)
    left_target_velocity = (turn_vel*WHEEL_BASE - 2*fwd_vel)/(-2 * WHEEL_RADIUS)
    right_target_velocity = (turn_vel*WHEEL_BASE + 2*fwd_vel)/(2 * WHEEL_RADIUS)

    left_velocity = (2 * np.pi * ((msg.left_delta / ENCODER_RESOLUTION)/GEAR_RATIO)) / dt
    right_velocity = (2 * np.pi * ((msg.right_delta / ENCODER_RESOLUTION)/GEAR_RATIO)) / dt

    #left_target_velocity_arr = np.append(left_target_velocity_arr, left_target_velocity)
    #right_target_velocity_arr = np.append(right_target_velocity_arr, right_target_velocity)
    #left_velocity_arr = np.append(left_velocity_arr, left_velocity)
    #right_velocity_arr = np.append(right_velocity_arr, right_velocity)
    left_target_velocity_arr.append(left_target_velocity)
    right_target_velocity_arr.append(right_target_velocity)
    left_velocity_arr.append(left_velocity)
    right_velocity_arr.append(right_velocity)

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
time.sleep(0.5)
command = mbot_motor_command_t()

command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
subscription = lc.subscribe("MBOT_ENCODERS", my_handler)

for i in range(100):
    lc.handle()

command.trans_v = 0.0
command.angular_v = 0
lc.publish("MBOT_MOTOR_COMMAND",command.encode())

#t = np.arange(left_target_velocity.shape[0])
t = list(range(0, len(left_target_velocity_arr)))
plt.plot(t, left_target_velocity_arr, 'r', t, left_velocity_arr, 'b')
plt.show()
plt.plot(t, right_target_velocity_arr, 'r', t, right_velocity_arr, 'b')
plt.show()