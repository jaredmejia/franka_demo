import os
from multiprocessing import Process, Queue
from time import strftime, localtime, sleep
import numpy as np

from franka_demo.demo_interfaces import print_and_cr
from franka_demo.addon import save_camdata_to_images

def add_logging_function(state):
    state.handlers['l'] = _press_logging
    state.onclose.append(clear_log_queue_on_quit)
    state.log_queue = Queue()
    state.is_logging_to = None

    # set up folder to save logs
    if not 'log_folder' in state.params:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = os.path.join(dir_path, '..', 'logs')
        print_and_cr(f"[INFO] Log to {dir_path}")
        state.params['log_folder'] = dir_path
    if not os.path.isdir(state.params['log_folder']):
        print_and_cr(f"[INFO] Create folder {state.params['log_folder']} for storing logs")
        os.mkdir(state.params['log_folder'])
    state.log_folder = state.params['log_folder']

def _press_logging(key_pressed, state):
    if state.is_logging_to:
        state.log_queue.put(None)
        state.is_logging_to = None
        print_and_cr(f"[LOGGING] Stop logging")
    else:
        state.is_logging_to = os.path.join(
            state.log_folder,
            strftime('%y-%m-%d-%H-%M-%S', localtime())
        )
        os.mkdir(state.is_logging_to)
        Process(
            target=start_logging,
            args=(state.is_logging_to, state.log_queue,
                  (state.cameras is not None))).start()
        print_and_cr(f"[LOGGING] Start logging to {state.is_logging_to}")

def start_logging(folder_name, q, log_camera):
  file_handler = open(os.path.join(folder_name, 'log.csv'), 'a')
  idx = 0
  while True:
    new_items = q.get(block=True)
    if new_items is None:
        file_handler.close()
        return
    for item in new_items[:-1]:
        if isinstance(item, np.ndarray):
            file_handler.write(np.array2string(item,
                precision=8, separator=' ', max_line_width=9999)[1:-1])
        else:
            file_handler.write(str(item))
        file_handler.write(',')
    # Last item is always the camera data
    if log_camera:
        save_camdata_to_images(new_items[-1], os.path.join(folder_name, str(idx)))
    file_handler.write('\n')
    idx += 1

def clear_log_queue_on_quit(state):
    state.log_queue.close()
    state.log_queue.join_thread()