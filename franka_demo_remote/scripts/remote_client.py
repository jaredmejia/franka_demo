import sys
import numpy as np
from time import time, sleep
import redis
from threading import Lock, Thread
from time import time, sleep
# from franka_demo_remote.demo_interfaces import print_and_cr
from franka_demo_remote.getch import getch
from franka_demo_remote.addon.camera import render_cam_state

REDIS_KEYBOARD_KEY = "franka-cmd"
REDIS_KEYBOARD_DUMMY_KEY = "-"
HANDLER_KEYS = ['r', 'p', 'h', 't', 'l', 'C', 'q']
KEYBOARD_CMD_SHAPE = 1
KEYBOARD_CMD_DTYPE = str

class RemoteState(object):
    def __init__(self, redis_store):
        self.redis_store = redis_store
        self.cameras = None
        self.quit = False

def receive_keyboard_cmd(state):
    # Keyboard Interface, running on a separate thread
    # print_and_cr("[INFO] Accepting keyboard commands, press 'h' for help.")
    res = getch()
    while res != 'q' and not state.quit: # Press q to quit
        if res in HANDLER_KEYS:
            redis_send_keyboardcmd(redis_store, res)
            # state.redis_store.set('processed', 0)
        sleep(0.01)
        res = getch()
    # print_and_cr("[INFO] Quitting the demo ...")
    state.quit = True
    redis_send_keyboardcmd(redis_store, 'q')
    # state.redis_store.set('processed', 0)
    return None

def redis_send_keyboardcmd(redis_store, keyboardcmd):
    redis_store.set(REDIS_KEYBOARD_KEY, keyboardcmd)

if __name__ == "__main__":
    HOST_IP = "172.26.114.61"
    redis_store = redis.Redis(HOST_IP)
    state = RemoteState(redis_store)

    keyboard_thread = Thread(target=receive_keyboard_cmd, name='Keyboard Thread', args=(state,))
    keyboard_thread.start()

    visual_thread = Thread(target=render_cam_state, name="Render camera states", args=(state,))
    visual_thread.start()

    keyboard_thread.join()
    visual_thread.join()
    # print_and_cr("[INFO] Demo Closed")

