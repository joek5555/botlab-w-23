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

old_x = []
old_y = []
old_theta = []
old_time = []


#left_target_velocity_arr = np.array([0])
#right_target_velocity_arr = np.array([0])
#left_velocity_arr = np.array([0])
#right_velocity_arr = np.array([0])

left_target_velocity_arr = []
right_target_velocity_arr = []
left_velocity_arr = []
right_velocity_arr = []

forward_vel_odom = []
forward_vel_set = []
angular_vel_odom = []
angular_vel_set =[]

def start_lc_handle(lc, stop_thread_func):
    event = threading.Event()
    event.clear()
    while True:
        lc.handle()
        if stop_thread_func():
            print("Killing thread")
            break

def encoder_msg_handle(channel, data):

    msg = mbot_encoder_t.decode(data)
    left_target_velocity = (turn_vel*WHEEL_BASE - 2*fwd_vel)/(-2 * WHEEL_RADIUS)
    right_target_velocity = (turn_vel*WHEEL_BASE + 2*fwd_vel)/(2 * WHEEL_RADIUS)

    left_velocity = (2 * np.pi * ((msg.left_delta / ENCODER_RESOLUTION)/GEAR_RATIO)) / dt
    right_velocity = (2 * np.pi * ((msg.right_delta / ENCODER_RESOLUTION)/GEAR_RATIO)) / dt

    left_target_velocity_arr.append(left_target_velocity)
    right_target_velocity_arr.append(right_target_velocity)
    left_velocity_arr.append(left_velocity)
    right_velocity_arr.append(right_velocity)

def odom_msg_handle(channel, data):

    msg = odometry_t.decode(data)

    if len(old_time) > 0:

        forward_vel_odom.append((np.sqrt((msg.x - old_x[0])**2 + (msg.y - old_y[0])**2)) * 1000000/ (msg.utime - old_time[0]))
        angular_vel_odom.append((msg.theta - old_theta[0]) * 1000000/ (msg.utime - old_time[0]))

        forward_vel_set.append(fwd_vel)
        angular_vel_set.append(turn_vel)
        old_time[0] = msg.utime
        old_x[0] = msg.x
        old_y[0] = msg.y
        old_theta[0] = msg.theta

    else:
        
        old_time.append(msg.utime)
        old_x.append(msg.x)
        old_y.append(msg.y)
        old_theta.append(msg.theta)
    
    



stop_thread = False 
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
subscription = lc.subscribe("MBOT_ENCODERS", encoder_msg_handle)
subscription = lc.subscribe("ODOMETRY", odom_msg_handle)
time.sleep(0.1)
lcm_thread = threading.Thread(target=start_lc_handle, args = (lc,lambda : stop_thread,))
lcm_thread.start()



fwd_vel = 0.25
turn_vel = 0.0
command = mbot_motor_command_t()
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(4.0)

fwd_vel = 0.0
turn_vel = 3.14
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(0.5)

fwd_vel = 0.25
turn_vel = 0.0
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(4.0)

fwd_vel = 0.0
turn_vel = 3.14
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(0.5)

fwd_vel = 0.25
turn_vel = 0.0
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(4.0)

fwd_vel = 0.0
turn_vel = 3.14
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(0.5)

fwd_vel = 0.25
turn_vel = 0.0
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(4.0)

fwd_vel = 0.0
turn_vel = 3.14
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(0.5)

fwd_vel = 0.0
turn_vel = 0.0
command.trans_v = fwd_vel
command.angular_v = turn_vel
lc.publish("MBOT_MOTOR_COMMAND",command.encode())
time.sleep(1.0)

print("set kill thread to true")
stop_thread = True
#lc_handle_thread.join()
time.sleep(1.0)
# plot the data
t = list(range(0, len(left_target_velocity_arr)))
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.subplots_adjust(hspace=0.05)
# code to break axis into two sections
ax1.plot(t, left_target_velocity_arr, 'r', t, left_velocity_arr, 'b')
ax2.plot(t, left_target_velocity_arr, 'r', t, left_velocity_arr, 'b')
ax1.set_ylim(4,8)
ax2.set_ylim(-8,-3)

ax1.spines.bottom.set_visible(False)
ax2.spines.top.set_visible(False)
ax1.xaxis.tick_top()
ax1.tick_params(labeltop=False)  # don't put tick labels at the top
ax2.xaxis.tick_bottom()
d = .5  # proportion of vertical to horizontal extent of the slanted line
kwargs = dict(marker=[(-1, -d), (1, d)], markersize=12,
              linestyle="none", color='k', mec='k', mew=1, clip_on=False)
ax1.plot([0, 1], [0, 0], transform=ax1.transAxes, **kwargs)
ax2.plot([0, 1], [1, 1], transform=ax2.transAxes, **kwargs)

ax1.legend(['left motor setpoint', 'left motor true value'])
#ax2.legend(['left motor setpoint', 'left motor true value'])
fig.savefig("left_motor_pid_square.png")
plt.show()

# plot right motor
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.subplots_adjust(hspace=0.05)
# code to break axis into two sections
ax1.plot(t, right_target_velocity_arr, 'r', t, right_velocity_arr, 'b')
#ax2.plot(t, right_target_velocity_arr, 'r', t, right_velocity_arr, 'b')
ax1.set_ylim(4,8)
ax2.set_ylim(-8, -3)

ax1.spines.bottom.set_visible(False)
ax2.spines.top.set_visible(False)
ax1.xaxis.tick_top()
ax1.tick_params(labeltop=False)  # don't put tick labels at the top
ax2.xaxis.tick_bottom()
d = .5  # proportion of vertical to horizontal extent of the slanted line
kwargs = dict(marker=[(-1, -d), (1, d)], markersize=12,
              linestyle="none", color='k', mec='k', mew=1, clip_on=False)
ax1.plot([0, 1], [0, 0], transform=ax1.transAxes, **kwargs)
ax2.plot([0, 1], [1, 1], transform=ax2.transAxes, **kwargs)

ax1.legend(['right motor setpoint', 'right motor true value'])
#ax2.legend(['right motor setpoint', 'right motor true value'])
fig.savefig("right_motor_pid_square.png")
plt.show()


t = list(range(0, len(forward_vel_odom)))
fig, ax = plt.subplots()
ax.plot(t, forward_vel_set, t, forward_vel_odom)
ax.legend(['desired vel', 'odometry'])
ax.set(xlabel='time', ylabel='vel m/s',
       title='forward_velocity error')
fig.savefig("forward_vel_error.png")
plt.show()

t = list(range(0, len(angular_vel_odom)))
fig, ax = plt.subplots()
ax.plot(t, angular_vel_set, t, angular_vel_odom)
ax.legend(['desired vel', 'odometry'])
ax.set(xlabel='time', ylabel='vel rad/s',
       title='angular_velocity error')
fig.savefig("angular_vel_error.png")
plt.show()