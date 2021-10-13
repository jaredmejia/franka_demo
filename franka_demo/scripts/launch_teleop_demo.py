import sys
import numpy as np
from time import time, sleep

from franka_demo.hardware_franka import get_args
from franka_demo.demo_interfaces import run_demo
from franka_demo.addon import add_teleop_function

def callback_func(state):
    add_teleop_function(state)

if __name__ == "__main__":

    args = get_args()

    run_demo(callback_func, params={
        'ip_address': args.server_ip,
    })