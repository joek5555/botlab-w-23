import numpy as np
import matplotlib.pyplot as plt
import os


path_to_odometry = '/home/pi/botlab-w-23/build/bin/odometry.txt'
path_to_desired_pose = '/home/pi/botlab-w-23/build/bin/desired_pose.txt'

with open(path_to_odometry, 'r') as file:
    odometry_lines = file.readlines()

with open(path_to_desired_pose, 'r') as file:
    desired_pose_lines = file.readlines()


# create arrays to store x, y, and z data
desired_x = np.zeros(len(desired_pose_lines))
desired_y = np.zeros(len(desired_pose_lines))
desired_theta = np.zeros(len(desired_pose_lines))
time_desired = np.arange(0, len(desired_pose_lines), 1)

odometry_x = np.zeros(len(odometry_lines))
odometry_y = np.zeros(len(odometry_lines))
odometry_theta = np.zeros(len(odometry_lines))
time_odometry = np.arange(0, len(odometry_lines), 1)

for i,desired_pose_str in enumerate(desired_pose_lines):

    desired_pose_str_list = desired_pose_str.split()
    desired_x[i] = desired_pose_str_list[0]
    desired_y[i] = desired_pose_str_list[1]
    desired_theta[i] = desired_pose_str_list[2]

for i,odometry_str in enumerate(odometry_lines):

    odometry_str_list = odometry_str.split()
    odometry_x[i] = odometry_str_list[0]
    odometry_y[i] = odometry_str_list[1]
    odometry_theta[i] = odometry_str_list[2]



fig, ax = plt.subplots()
ax.plot(time_desired, desired_x, time_odometry, odometry_x)
ax.legend(['desired pose', 'odometry'])
ax.set(xlabel='counts', ylabel='x (m)',
       title='Odometry vs desired path x')
fig.savefig("odom_vs_desired_x.png")
plt.show()

fig, ax = plt.subplots()
ax.plot(time_desired, desired_y, time_odometry, odometry_y)
ax.legend(['desired pose', 'odometry'])
ax.set(xlabel='counts', ylabel='y (m)',
       title='Odometry vs desired path y')
fig.savefig("odom_vs_desired_y.png")
plt.show()

fig, ax = plt.subplots()
ax.plot(time_desired, desired_theta, time_odometry, odometry_theta)
ax.legend(['desired pose', 'odometry'])
ax.set(xlabel='counts', ylabel='theta (rad)',
       title='Odometry vs desired path theta')
fig.savefig("odom_vs_desired_theta.png")
plt.show()