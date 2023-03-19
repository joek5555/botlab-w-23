import time
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_pid_gains_t
import numpy as np
import matplotlib.pyplot as plt
import argparse


def parse_args():
    parser = argparse.ArgumentParser(description='change mbot pid values')
    parser.add_argument('--left_p', type=float, help='Which task are you running?',
                        default=0.04)
    parser.add_argument('--left_i', type=float, help='Which task are you running?',
                        default=0.06)
    parser.add_argument('--left_d', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--left_d_hz', type=float, help='Which task are you running?',
                        default=25.0)
    parser.add_argument('--right_p', type=float, help='Which task are you running?',
                        default=0.04)
    parser.add_argument('--right_i', type=float, help='Which task are you running?',
                        default=0.06)
    parser.add_argument('--right_d', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--right_d_hz', type=float, help='Which task are you running?',
                        default=25.0)

    parser.add_argument('--fwd_p', type=float, help='Which task are you running?',
                        default=1.0)
    parser.add_argument('--fwd_i', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--fwd_d', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--fwd_d_hz', type=float, help='Which task are you running?',
                        default=10.0)
    parser.add_argument('--ang_p', type=float, help='Which task are you running?',
                        default=1.0)
    parser.add_argument('--ang_i', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--ang_d', type=float, help='Which task are you running?',
                        default=0.0)
    parser.add_argument('--ang_d_hz', type=float, help='Which task are you running?',
                        default=10.0)

    
    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    time.sleep(0.1)
    pid = mbot_pid_gains_t()

    pid.motor_a_kp = args.left_p
    pid.motor_a_ki = args.left_i
    pid.motor_a_kd = args.left_d
    pid.motor_a_Tf = args.left_d_hz

    pid.motor_c_kp = args.right_p
    pid.motor_c_ki= args.right_i
    pid.motor_c_kd = args.right_d
    pid.motor_c_Tf = args.right_d_hz

    pid.bf_trans_kp = args.fwd_p
    pid.bf_trans_ki = args.fwd_i
    pid.bf_trans_kd = args.fwd_d
    pid.bf_trans_Tf = args.fwd_d_hz

    pid.bf_rot_kp= args.ang_p
    pid.bf_rot_ki = args.ang_i
    pid.bf_rot_kd = args.ang_d
    pid.bf_rot_Tf  = args.ang_d_hz
    
    lc.publish("MBOT_PIDS",pid.encode())
    time.sleep(0.5)


if __name__ == '__main__':
    main()