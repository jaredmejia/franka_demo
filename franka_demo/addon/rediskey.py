import numpy as np
import redis
from time import sleep
from threading import Thread

from franka_demo.demo_interfaces import print_and_cr

REDIS_KEYBOARD_KEY = "franka-cmd"
REDIS_KEYBOARD_DUMMY_KEY = "-"

def add_rediskey_function(state):
    state.redis_store.set(REDIS_KEYBOARD_KEY, REDIS_KEYBOARD_DUMMY_KEY)
    state.rediskey_thread = Thread(
        target=redis_keyboard_proc, name='Redis keyboard',
        args=(state,), daemon=True)
    state.rediskey_thread.start()
    state.onclose.append(close_redis_keyboard_proc)

def redis_keyboard_proc(state):
    while not state.quit:
        res = state.redis_store.get(REDIS_KEYBOARD_KEY).decode("utf-8")
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
        sleep(0.1)

def close_redis_keyboard_proc(state):
    state.rediskey_thread.join()