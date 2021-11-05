import numpy as np
from time import time

from franka_demo_remote.demo_interfaces import print_and_cr

SINWAVE_JOINT = 6
SINWAVE_MAGNITUDE = 0.5     # magnitude of sine wave (rad)
SINWAVE_PERIOD = 2.0        # period of sine wave (s)

def add_sinwave_function(state):
    state.handlers['s'] = _press_sinwave
    state.modes['sinwave'] = __cmd_sinwave

def _press_sinwave(key_pressed, state):
    print_and_cr(f"[INFO] Enter Sin Wave Motion on joint idx {SINWAVE_JOINT}")
    state.ref_state = np.array(state.robostate)
    state.ref_timestamp = time()
    state.mode = 'sinwave'

def __cmd_sinwave(state, timestamp):
    cmd = np.array(state.ref_state)
    cmd[SINWAVE_JOINT] += SINWAVE_MAGNITUDE * \
        np.sin(np.pi * (timestamp - state.ref_timestamp) / SINWAVE_PERIOD)
    return cmd