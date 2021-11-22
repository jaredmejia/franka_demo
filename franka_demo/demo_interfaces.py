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

<<<<<<< HEAD
from franka_demo.hardware_franka import FrankaArm, FrankaArmWithGripper, JointPDPolicy
from multiprocessing import Process

=======
from franka_demo.hardware_franka import FrankaArm, FrankaArmWithGripper, FrankaArmWithRobotiQGripper, JointPDPolicy
>>>>>>> a7cc05797436c1ef55ba8e7030a32bb0c9681fd0
from franka_demo.hardware_dummy import DummyFrankaArm
from franka_demo.utils import print_and_cr

from .getch import getch

STATE_UPDATE_FREQ = 40                     # Refresh joint position at 40Hz
CMD_EVERY_ITER = 1                          # Send command at 40/1 = 40Hz
REDIS_STATE_KEY = 'robostate'
REDIS_CMD_KEY = 'robocmd'
CMD_DTYPE = np.float64         # np.float64 for C++ double; np.float32 for float

<<<<<<< HEAD
# REDIS_KEYBOARD_CMD_KEY = "keyboardcmd" # replaced with below
REDIS_KEYBOARD_KEY = "franka-cmd"
REDIS_KEYBOARD_DUMMY_KEY = "-"


def print_and_cr(msg): sys.stdout.write(msg + '\r\n')

=======
>>>>>>> a7cc05797436c1ef55ba8e7030a32bb0c9681fd0
class State(object):
    def __init__(self, franka, redis_store):
        self.franka = franka
        self.cmd_shape = franka.CMD_SHAPE
        self.CMD_DELTA_HIGH = np.array([0.1] * franka.CMD_SHAPE)
        self.CMD_DELTA_LOW = np.array([-0.1] * franka.CMD_SHAPE)

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

    def redis_receive_command(self):
        return np.array(np.frombuffer(self.redis_store.get(REDIS_CMD_KEY), dtype=CMD_DTYPE).reshape(8))[:self.cmd_shape]

def init_robot(ip_address, gripper):
    print_and_cr(f"[INFO] Try connecting to Franka robot at {ip_address} ...")
    if ip_address == "0":
        franka = DummyFrankaArm(name="Dummy", ip_address=None)
    elif gripper == False:
        franka = FrankaArm(name="Franka-Demo", ip_address=ip_address)
    else:
        franka = FrankaArmWithRobotiQGripper(name="Franka-Demo-Gripper", ip_address=ip_address)
    franka.reset()
    franka.connect(policy=franka.default_policy())
    print_and_cr(f"[INFO] Connected to Franka arm")
    redis_store = redis.Redis()
    return State(franka, redis_store)

def redis_send_states(redis_store, robostate):
    redis_store.set(REDIS_STATE_KEY, robostate.tobytes())

def redis_send_dummy_command(redis_store, robopos):
    redis_store.set(REDIS_CMD_KEY, robopos.tobytes())


def redis_receive_keyboardcmd(redis_store):
    return redis_store.get(REDIS_KEYBOARD_KEY).decode("utf-8")

    

# ------------------------------------------------------------------------------
# Example Functions

def _press_reset(key_pressed, state):
    state.mode = 'reset'

def __cmd_reset(state, timestamp):
    print_and_cr(f"[INFO] Resetting robot. Takes about 4 sec. Do not move ..")
    state.franka.reset()
    print_and_cr(f"[INFO] Franka is reset, sending our controller")
    state.franka.connect(policy=state.franka.default_policy())
    state.mode = 'idle'
    return None

def _press_print(key_pressed, state):
    state.print_state = True

def _press_idle(key_pressed, state):
    state.mode = 'idle'
    print_and_cr("Enter idle mode")

def __cmd_idle(state, timestamp):
    return None

def _press_start(key_pressed, state):
    state.mode = 'start'
    print_and_cr("Return to home position.")

def __cmd_start(state, timestamp):
    clipped_cmd = np.clip(state.franka.START_POSITION,
        state.robostate + state.CMD_DELTA_LOW,
        state.robostate + state.CMD_DELTA_HIGH)
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

def keyboard_proc(state, remote=False):
    # Keyboard Interface, running on a separate thread
    
    print_and_cr("[INFO] Accepting keyboard commands, press 'h' for help.")
    while not state.quit:
        if remote:
            res = redis_receive_keyboardcmd(state.redis_store)
        else:
            res = getch()
        if res != REDIS_KEYBOARD_DUMMY_KEY:
            state.redis_store.set(REDIS_KEYBOARD_KEY, REDIS_KEYBOARD_DUMMY_KEY)
            if res in state.handler_keys:
                print_and_cr(f"[REDISKEY] Received {res}")
                state.handlers[res](res, state)
            elif res == 'q':
                print_and_cr(f"[INFO] Received redis key to quit")
                state.quit = True
            else:
                print_and_cr(f"[WARNING] Invalid redis command {res}")
        sleep(0.01)
    print_and_cr("[INFO] Quitting the demo ...")
    state.quit = True
    return None

def run_demo(callback_to_install_func=None, params={}):
    # Command thread
    if 'gripper' not in params: params['gripper'] = True
    state = init_robot(params['ip_address'], params['gripper'])

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

    keyboard_thread = Thread(target=keyboard_proc, name='Keyboard Thread', args=(state, state.params['remote']))
    keyboard_thread.start()

    ts = time()
    period = 1 / STATE_UPDATE_FREQ
    ts_counter = 0

    while not state.quit:
        state.robostate = np.array(state.franka.get_sensors_offsets(), dtype=CMD_DTYPE)
        redis_send_states(state.redis_store, state.robostate)

        #if ts_counter % 40 == 0:
        #    print_and_cr(f"{state.robostate}")

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

    print_and_cr("[INFO] Closing the demo.")

    for func in state.onclose:
        func(state)

    state.franka.close()
    print_and_cr("[INFO] Demo Closed")