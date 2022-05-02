import sys
import numpy as np
from time import time, sleep
import redis
from threading import Lock, Thread
from time import time, sleep
from franka_demo_remote.utils import print_and_cr
from franka_demo_remote.getch import getch
from franka_demo_remote.remote_utils import render_cam_state

REDIS_KEYBOARD_KEY = "franka-cmd"
REDIS_KEYBOARD_DUMMY_KEY = "-"
HANDLER_KEYS = ['r', 'p', 'h', 't', 'l', 'C', 'q', '.', '/']
KEYBOARD_CMD_SHAPE = 1
KEYBOARD_CMD_DTYPE = str

class RemoteState(object):
    def __init__(self, redis_store):
        self.redis_store = redis_store
        self.cameras = None
        self.quit = False
def _remote_press_reset(key_pressed, state):
    # need status
    print_and_cr(f"[INFO] Resetting robot. Takes about 4 sec. Do not move ..")
    print_and_cr(f"[INFO] Franka is reset, sending our controller")
    pass

def _remote_press_print(key_pressed, state):
    print_and_cr(f"[INFO] Current state {state.robostate}")

def _remote_press_help(key_pressed, state):
    print_and_cr("Keypress Handlers:") 
    keys = sorted(state.handlers.keys())
    for k in keys:
        print_and_cr("\t%s - %s" % (k, state.handlers[k].__name__.replace("press","").replace("remote","").replace("_", " ").strip()))
    print_and_cr("\tq - quit the demo")
    print_and_cr("Robot Modes:")
    modes = sorted(state.modes.keys())
    for m in modes:
        print_and_cr("\t%s" % m)

def _remote_press_teleop(key_pressed, state):
    print_and_cr(f"[INFO] Enter Teleop")

def _remote_press_logging(key_pressed, state):
    # need status
    pass


def _remote_press_debug_update_camera_fps(key_pressed, state):
    # need num_camera status
    pass
    

def _remote_press_idle(key_pressed, state):
    print_and_cr("Enter idle mode")

def _remote_press_start(key_pressed, state):
    print_and_cr("Return to home position.")

def setup_handlers(state):
    handlers = {}   # Handle key pressing event
    handlers['r'] = _remote_press_reset
    handlers['p'] = _remote_press_print
    handlers['h'] = _remote_press_help
    handlers['t'] = _remote_press_teleop
    handlers['l'] = _remote_press_logging
    handlers['C'] = _remote_press_debug_update_camera_fps
    handlers['.'] = _remote_press_idle
    handlers['/'] = _remote_press_start
    state.handlers = handlers

def receive_keyboard_cmd(state):
    # Keyboard Interface, running on a separate thread
    print_and_cr("[INFO] Accepting keyboard commands, press 'h' for help.")
    res = getch()
    while res != 'q' and not state.quit: # Press q to quit
        if res in HANDLER_KEYS:
            print_and_cr(f"[REDISKEY] Received {res}")
            redis_send_keyboardcmd(redis_store, res)
            state.handlers[res](res, state)
            # state.redis_store.set('processed', 0)
        sleep(0.01)
        res = getch()
    print_and_cr("[INFO] Quitting the client demo ...")
    state.quit = True
    # redis_send_keyboardcmd(redis_store, 'q')
    # state.redis_store.set('processed', 0)
    return None

def redis_send_keyboardcmd(redis_store, keyboardcmd):
    redis_store.set(REDIS_KEYBOARD_KEY, keyboardcmd)

def redis_receive_command(self):
    return np.array(np.frombuffer(self.redis_store.get(REDIS_CMD_KEY), dtype=CMD_DTYPE).reshape(8))[:self.cmd_shape]

if __name__ == "__main__":
    HOST_IP = "172.26.114.61"
    redis_store = redis.Redis(HOST_IP)
    state = RemoteState(redis_store)
    setup_handlers(state)

    keyboard_thread = Thread(target=receive_keyboard_cmd, name='Keyboard Thread', args=(state,))
    keyboard_thread.start()

    visual_thread = Thread(target=render_cam_state, name="Render camera states", args=(state,))
    visual_thread.start()

    keyboard_thread.join()
    visual_thread.join()
    print_and_cr("[INFO] Demo Closed")

