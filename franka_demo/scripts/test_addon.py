import sys
import numpy as np
from time import time, sleep

from franka_demo.hardware_franka import get_args
from franka_demo.demo_interfaces import run_demo
from franka_demo.addon import add_teleop_function, add_logging_function, add_sinwave_function, add_replay_function, add_camera_function

def callback_func(state):
    add_teleop_function(state)
    add_logging_function(state)
    add_sinwave_function(state)
    add_replay_function(state)
    add_camera_function(state)

if __name__ == "__main__":

    args = get_args()

    run_demo(callback_func, params={
        'ip_address': args.server_ip,
        'log_folder': 'logs',
        'replay_filename': 'franka_demo/logs/21-10-14-19-50-07.csv'
    })