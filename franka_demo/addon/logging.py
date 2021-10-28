import os
from multiprocessing import Process, Queue
from time import strftime, localtime, sleep
import numpy as np

from franka_demo.demo_interfaces import print_and_cr

def add_logging_function(state):
    state.handlers['l'] = _press_logging
    state.onclose.append(terminate_logging)
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

def terminate_logging(state):
    if state.is_logging_to is not None:
        state.log_queue.put(None)
        state.running_logger.join()
        state.is_logging_to = None
        if hasattr(state, 'cameras') and state.cameras is not None:
            state.cameras.close_logger(state)
        print_and_cr(f"[LOGGING] Stop logging")

def _press_logging(key_pressed, state):
    if state.is_logging_to:
        terminate_logging(state)
    else:
        state.is_logging_to = os.path.join(
            state.log_folder,
            strftime('%y-%m-%d-%H-%M-%S', localtime())
        )
        os.mkdir(state.is_logging_to)
        if hasattr(state, 'cameras') and state.cameras is not None:
            state.cameras.launch_logger(state)
        state.running_logger = Process(
            target=start_logging,
            args=(state.is_logging_to, state.log_queue,
                  (hasattr(state, 'cameras') and state.cameras is not None)))
        state.running_logger.start()
        print_and_cr(f"[LOGGING] Start logging to {state.is_logging_to}")

def start_logging(folder_name, q, log_camera):
  file_handler = open(os.path.join(folder_name, 'log.csv'), 'a')
  idx = 0
  while True:
    new_items = q.get(block=True)
    if new_items is None:
        file_handler.close()
        return
    simple_save_items = new_items[:-1] if log_camera else new_items
    for item in simple_save_items:
        if isinstance(item, np.ndarray):
            file_handler.write(np.array2string(item,
                precision=8, separator=' ', max_line_width=9999)[1:-1])
        else:
            file_handler.write(str(item))
        file_handler.write(',')
    if log_camera:
        for cam in new_items[-1]:
            for info in cam:
                file_handler.write(f"{info}-")
    file_handler.write('\n')
    idx += 1

def clear_log_queue_on_quit(state):
    state.log_queue.close()
    state.log_queue.join_thread()