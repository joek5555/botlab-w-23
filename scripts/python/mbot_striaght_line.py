import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t
from mbot_lcm_msgs import mbot_encoder_t
import numpy as np
import matplotlib.pyplot as plt
import threading

obj = time.gmtime(0)
epoch = time.asctime(obj)

fwd_vel = 0.0 # 0.25 m/s
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

def start_lc_handle(lc, stop_thread_func):
    while True:
        lc.handle()
        if stop_thread_func():
            print("Killing thread")
            break

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

stop_thread = False 
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
subscription = lc.subscribe("MBOT_ENCODERS", my_handler)
time.sleep(0.1)
lc_handle_thread = threading.Thread(target=start_lc_handle, args = (lc,lambda : stop_thread,))
lc_handle_thread.start()


fwd_vel = 0.1
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

fwd_vel = 0.0
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(5.0)


fwd_vel = 0.2
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

fwd_vel = 0.0
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(5.0)

fwd_vel = 0.25
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

fwd_vel = 0.0
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(5.0)


fwd_vel = 0.3
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(10.0)

fwd_vel = 0.0
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())

print("set kill thread to true")
stop_thread = True
#lc_handle_thread.join()
time.sleep(1.0)


#t = np.arange(left_target_velocity.shape[0])
t = list(range(0, len(left_target_velocity_arr)))

plt.plot(t, left_target_velocity_arr, 'r', t, left_velocity_arr, 'b')
plt.legend(['left motor setpoint', 'left motor true value'])
plt.savefig("left_motor_pid_straight.png")
plt.show()


plt.plot(t, right_target_velocity_arr, 'r', t, right_velocity_arr, 'b')
plt.legend(['left motor setpoint', 'left motor true value'])
plt.savefig("left_motor_pid_straight.png")
plt.show()