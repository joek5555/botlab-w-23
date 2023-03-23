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

time.sleep(30.0)

print("set kill thread to true")
stop_thread = True
#lc_handle_thread.join()
time.sleep(1.0)
# plot the data



fig, ax = plt.subplots()
t = list(range(0, len(x_odom)))

ax.plot(t, x_odom, t, y_odom)
ax.legend(['x', 'y'])
ax.set(xlabel='t', ylabel='m',
       title='x y Odom vs time')
fig.savefig("x_y_odom_vs_time.png")
plt.show()

fig, ax = plt.subplots()
ax.plot(t, theta_odom)
ax.legend(['theta'])
ax.set(xlabel='t', ylabel='rad',
       title='theta Odom vs time')
fig.savefig("theta_odom_vs_time.png")
plt.show()