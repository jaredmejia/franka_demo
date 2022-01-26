import os
from multiprocessing import Process, Queue
from time import strftime, localtime, sleep
import numpy as np

from franka_demo.utils import print_and_cr, colors

def add_logging_function(state):
    state.log_queue = Queue()
    state.is_logging_to = None
    state.logger_process = Process(
        target=start_logging,
        args=(state.log_queue,))
    state.logger_process.start()

    state.handlers['l'] = _press_logging
    state.onclose.append(terminate_logging)
    state.onclose.append(clear_log_queue_on_quit)

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
        state.is_logging_to = None
        #if hasattr(state, 'cameras') and state.cameras is not None:
        #    state.cameras.terminate_logging(state)
        print_and_cr(f"[LOGGING] Stop logging")

def _press_logging(key_pressed, state):
    if state.is_logging_to:
        terminate_logging(state)
    else:
        new_log_path = os.path.join(
            state.log_folder,
            strftime('%y-%m-%d-%H-%M-%S', localtime())
        )
        os.mkdir(new_log_path)
        state.is_logging_to = new_log_path
        state.log_queue.put(new_log_path)

        #if hasattr(state, 'cameras') and state.cameras is not None:
        #    state.cameras.start_logging(state)
        print_and_cr(f"[LOGGING] Start logging to {state.is_logging_to}")

def start_logging(q):
    while True:
        folder_name = q.get(block=True)
        if folder_name is None: break

        try:
            file_handler = open(os.path.join(folder_name, 'log.csv'), 'a')
        except Exception as e:
            print_and_cr(f"{colors.bg.red}Cannot open file for writing log?")
        print_and_cr(f"{colors.bg.green}[LOGGING] Created log.csv in {folder_name}")
        idx = 0
        while True:
            new_items = q.get(block=True)
            if new_items is None:
                file_handler.close()
                print_and_cr(f"{colors.reset}[LOGGING] Close log.csv in {folder_name}")
                break
            simple_save_items = new_items[:-1]
            for item in simple_save_items:
                if isinstance(item, np.ndarray):
                    file_handler.write(np.array2string(item,
                        precision=8, separator=' ', max_line_width=9999)[1:-1])
                else:
                    file_handler.write(str(item))
                file_handler.write(',')
            if new_items[-1] is not None:
                file_handler.write("-".join(map(str, new_items[-1])))
            else:
                file_handler.write('None')
            file_handler.write('\n')
            idx += 1

def clear_log_queue_on_quit(state):
    state.log_queue.put(None)
    state.log_queue.close()
    state.log_queue.join_thread()