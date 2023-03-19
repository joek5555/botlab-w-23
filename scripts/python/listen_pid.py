import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_pid_gains_t
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
    print("pid loop")
    msg = mbot_pid_gains_t.decode(data)
    print(msg.motor_a_kp)



stop_thread = False 
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
subscription = lc.subscribe("MBOT_PIDS", my_handler)
time.sleep(0.1)
lc_handle_thread = threading.Thread(target=start_lc_handle, args = (lc,lambda : stop_thread,))
lc_handle_thread.start()