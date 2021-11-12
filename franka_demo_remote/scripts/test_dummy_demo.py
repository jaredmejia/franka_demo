import sys
import numpy as np
from time import time, sleep

from franka_demo_remote.hardware_franka import get_args
from franka_demo_remote.demo_interfaces import run_demo

def _press_step(key_pressed, state):
    state.mode = 'step'

def __cmd_step(state, timestamp):
    joint_pos_desired = np.array(state.robostate)
    joint_pos_desired[6] += 0.05
    state.mode = 'idle'
    return joint_pos_desired

def callback_func(state):
    state.handlers['s'] = _press_step
    state.modes['step'] = __cmd_step

if __name__ == "__main__":

    args = get_args()

    run_demo(callback_func, params={
        'ip_address': args.server_ip,
    })