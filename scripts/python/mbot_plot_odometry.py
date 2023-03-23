import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t
from mbot_lcm_msgs import mbot_encoder_t
from mbot_lcm_msgs import odometry_t
import numpy as np
import matplotlib.pyplot as plt
import threading

fwd_vel = 0.0# 0.25 m/s
turn_vel = 0.0 # pi rad/sec
dt = 0.05

WHEEL_BASE = 0.15
WHEEL_RADIUS = 0.04
ENCODER_RESOLUTION = 20
GEAR_RATIO = 78.0


#left_target_velocity_arr = np.array([0])
#right_target_velocity_arr = np.array([0])
#left_velocity_arr = np.array([0])
#right_velocity_arr = np.array([0])

x_odom = []
y_odom = []
theta_odom = []


def start_lc_handle(lc, stop_thread_func):
    while True:
        lc.handle()
        if stop_thread_func():
            print("Killing thread")
            break

def my_handler(channel, data):

    msg = odometry_t.decode(data)

    x_odom.append(msg.x)
    y_odom.append(msg.y)
    theta_odom.append(msg.theta)

stop_thread = False 
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
subscription = lc.subscribe("ODOMETRY", my_handler)
time.sleep(0.1)
lc_handle_thread = threading.Thread(target=start_lc_handle, args = (lc,lambda : stop_thread,))
lc_handle_thread.start()


# fwd_vel = 0.25
# turn_vel = 0.0
# command = mbot_motor_command_t()
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(4.0)

# fwd_vel = 0.0
# turn_vel = 3.14
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(0.5)

# fwd_vel = 0.25
# turn_vel = 0.0
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(4.0)

# fwd_vel = 0.0
# turn_vel = 3.14
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(0.5)

# fwd_vel = 0.25
# turn_vel = 0.0
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(4.0)

# fwd_vel = 0.0
# turn_vel = 3.14
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(0.5)

# fwd_vel = 0.25
# turn_vel = 0.0
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(4.0)

# fwd_vel = 0.0
# turn_vel = 3.14
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(0.5)

# fwd_vel = 0.0
# turn_vel = 0.0
# command.trans_v = fwd_vel
# command.angular_v = turn_vel
# lc.publish("MBOT_MOTOR_COMMAND",command.encode())
# time.sleep(1.0)

time.sleep(60.0)
print("set kill thread to true")
stop_thread = True
#lc_handle_thread.join()
time.sleep(1.0)
# plot the data

x_desired = [0, 0.61, 0.61, 1.22, 1.22, 1.83, 1.83, 2.44, 2.44, 3.05]
y_desired = [0, 0, -0.61, -0.61, 0.61, 0.61, -0.61, -0.61, 0, 0]

fig, ax = plt.subplots()
ax.plot(x_desired, y_desired, x_odom, y_odom)
ax.legend(['desired path', 'odometry'])
ax.set(xlabel='x (m)', ylabel='y (m)',
       title='Odometry vs desired path')
fig.savefig("odometry_plot.png")
plt.show()
