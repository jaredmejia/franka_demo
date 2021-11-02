# ==============================================================================
# Franka Demo
#   By Liyiming Ke (liyiming.ke@gmail)
# ==============================================================================

import sys
import numpy as np
import redis
import torch
from threading import Lock, Thread
from time import time, sleep

from franka_demo.hardware_franka import FrankaArm, JointPDPolicy
from franka_demo.hardware_dummy import DummyFrankaArm
from franka_demo import getch

STATE_UPDATE_FREQ = 200                     # Refresh joint position at 200Hz
CMD_EVERY_ITER = 5                          # Send command at 200/5 = 40Hz
REDIS_STATE_KEY = 'robostate'
REDIS_CMD_KEY = 'robocmd'
CMD_SHAPE = 7
CMD_DTYPE = np.float64  # np.float64 for C++ double; np.float32 for float
CMD_DELTA_HIGH = np.array([0.1] * CMD_SHAPE)
CMD_DELTA_LOW = np.array([-0.1] * CMD_SHAPE)
START_POSITION = np.array([-0.1422354, -0.02149742, -0.04364768, -2.07073975, 0.06118893, 0.42122769, -1.71912813])

def print_and_cr(msg): sys.stdout.write(msg + '\r\n')

class State(object):
    def __init__(self, franka, redis_store):
        self.franka = franka
        self.redis_store = redis_store
        self.cameras = None
        self.quit = False
        self.mode = 'idle'
        self._mutex = Lock() # Not in use
        self.print_state = False
        self.is_logging_to = None

    def lock(self):
        self._mutex.acquire()

    def unlock(self):
        self._mutex.release()

def init_robot(ip_address):
    print_and_cr(f"[INFO] Try connecting to Franka robot at {ip_address} ...")
    franka = FrankaArm(name="Franka-Demo", ip_address=ip_address) \
        if ip_address != "0" \
        else DummyFrankaArm(name="Dummy", ip_address=None)
    franka.reset()
    franka.connect(policy=franka.default_policy(1.5, 1.5))
    print_and_cr(f"[INFO] Connected to Franka arm")
    redis_store = redis.Redis()
    return State(franka, redis_store)

def redis_send_states(redis_store, robostate):
    redis_store.set(REDIS_STATE_KEY, robostate.tobytes())

def redis_send_dummy_command(redis_store, robopos):
    redis_store.set(REDIS_CMD_KEY, robopos.tobytes())

def redis_receive_command(redis_store):
    return np.array(np.frombuffer(redis_store.get(REDIS_CMD_KEY), dtype=CMD_DTYPE).reshape(CMD_SHAPE))

# ------------------------------------------------------------------------------
# Example Functions

def _press_reset(key_pressed, state):
    state.mode = 'reset'

def __cmd_reset(state, timestamp):
    print_and_cr(f"[INFO] Resetting robot. Takes about 4 sec. Do not move ..")
    state.franka.reset()
    print_and_cr(f"[INFO] Franka is reset, sending our controller")
    state.franka.connect(policy=state.franka.default_policy(1.0, 1.0))
    state.mode = 'idle'
    return None

def _press_print(key_pressed, state):
    state.print_state = True

def _press_idle(key_pressed, state):
    state.mode = 'idle'

def __cmd_idle(state, timestamp):
    return None

def _press_start(key_pressed, state):
    state.mode = 'start'

def __cmd_start(state, timestamp):
    clipped_cmd = np.clip(START_POSITION, state.robostate+CMD_DELTA_LOW, state.robostate+CMD_DELTA_HIGH)
    return clipped_cmd

def _press_help(key_pressed, state):
    print_and_cr("Keypress Handlers:")
    keys = sorted(state.handlers.keys())
    for k in keys:
        print_and_cr("\t%s - %s" % (k, state.handlers[k].__name__.replace("press","").replace("_", " ").strip()))
    print_and_cr("\tq - quit the demo")
    print_and_cr("Robot Modes:")
    modes = sorted(state.modes.keys())
    for m in modes:
        print_and_cr("\t%s" % m)


# ------------------------------------------------------------------------------

def keyboard_proc(state):
    # Keyboard Interface, running on a separate thread
    print_and_cr("[INFO] Accepting keyboard commands, press 'h' for help.")
    res = getch()
    while res != 'q' and not state.quit: # Press q to quit
        if res in state.handler_keys:
            state.handlers[res](res, state)
        sleep(0.01)
        res = getch()
    print_and_cr("[INFO] Quitting the demo ...")
    state.quit = True
    return None

def run_demo(callback_to_install_func=None, params={}):
    # Command thread
    state = init_robot(params['ip_address'])

    handlers = {}   # Handle key pressing event
    modes = {}      # Generate command depending on the mode
    onclose = []    # Clean up on closing the demo

    handlers['p'] = _press_print
    handlers['h'] = _press_help

    handlers['r'] = _press_reset
    modes['reset'] = __cmd_reset

    handlers['.'] = _press_idle
    modes['idle'] = __cmd_idle

    handlers['/'] = _press_start
    modes['start'] = __cmd_start

    state.handlers = handlers
    state.modes = modes
    state.onclose = onclose
    state.params = params

    if callback_to_install_func is not None:
        # Install additional functions, by add handlers / modes / onclose
        callback_to_install_func(state)

    state.handler_keys = state.handlers.keys()
    state.mode_keys = state.modes.keys()

    keyboard_thread = Thread(target=keyboard_proc, name='Keyboard Thread', args=(state,))
    keyboard_thread.start()

    ts = time()
    period = 1 / STATE_UPDATE_FREQ
    ts_counter = 0

    while not state.quit:
        state.robostate = np.array(state.franka.get_sensors_offsets(), dtype=CMD_DTYPE)
        redis_send_states(state.redis_store, state.robostate)

        if ts_counter % CMD_EVERY_ITER == 0:

            try:
                if state.franka.robot.get_previous_interval().end != -1:
                    print_and_cr("[WARNING] Custom Controller died")
                    print_and_cr("[WARNING] Resetting the robot")
                    state.mode = 'reset'
            except Exception as e:
                print_and_cr("[ERROR] Unable to communicate with Polymetis server")
                print_and_cr("Last known robostate {state.robostate}")
                print_and_cr("[ERROR] Shutting down demo")
                print(e)
                state.quit = True
                break

            if state.print_state:
                print_and_cr(f"[INFO] Current state {state.robostate}")
                state.print_state = False

            cam_data = None
            if state.cameras:
                cam_state = state.cameras.get_data()
                cam_data = [cam_state[c][1] for c in sorted(cam_state.keys())]
                # [cam0rgb timestamp, cam0depth ts, cam1rgb, cam1depth, ...]

            current_mode = state.mode
            assert current_mode in state.mode_keys
            command = state.modes[current_mode](state, time())

            if command is not None:
                state.franka.apply_commands_offsets(command)

            if state.is_logging_to:
                state.log_queue.put((
                    time(), state.robostate, command, cam_data
                ))

        ts_counter += 1
        sleep_till = ts + ts_counter * period
        sleep(max(sleep_till - time(), 0))

    for func in state.onclose:
        func(state)

    state.franka.close()
    keyboard_thread.join()
    print_and_cr("[INFO] Demo Closed")