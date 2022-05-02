import os
import numpy as np
import pickle
from franka_demo_remote.utils import print_and_cr
from franka_demo_remote.demo_interfaces import CMD_DTYPE
from franka_demo_remote.addon.logger import terminate_logging
def add_replay_function(state):
    state.handlers['p'] = _press_replay   # close-loop replay
    state.handlers['o'] = _press_replay   # open-loop replay
    state.modes['replay'] = __cmd_replay

    # state.handlers['p']('p', state) #hack
def _press_replay(key_pressed, state):
    # determine which file to replay
    if 'replay_filename' not in state.params:
        state.params['replay_filename'] = 'redis'
    if state.params['replay_filename'] == 'redis':
        replay_filename = state.redis_store.get('replay_filename')
        print_and_cr(f"[INFO] Obtained replay filename from redis: {replay_filename}")
    elif os.path.isfile(state.params['replay_filename']):
        replay_filename = state.params['replay_filename']
    else:
        print_and_cr(f"[ERROR] Cannot find the file for replay at {state.params['replay_filename']}")
        return
    try:
        state.replay_openloop = (key_pressed == 'o')
        state.replay_delta = ('delta' in replay_filename)
        if not state.replay_openloop and state.replay_delta:
            print_and_cr(f"[INFO] Close-loop replay delta-traj is equivalent to close-loop replay abs")
            state.replay_delta = False
        state.replay_commands, state.replay_start_state = load_pickle_commands(
            replay_filename,
            is_openloop=state.replay_openloop
        )
        state.replay_commands_length = len(state.replay_commands)
    except Exception as e:
        print_and_cr(f"[ERROR] Failed to load replay file at {replay_filename}")
        print(e)
        return
    state.replay_counter = 0
    print_and_cr(f"[INFO] Replaying commands from {replay_filename}")
    state.mode = 'replay'


def __cmd_replay(state, timestamp):
    if state.replay_counter == state.replay_commands_length:
        if state.is_logging_to is not None:
            # state.log_queue.put(None)
            # reward = input("Enter reward for traj: ")
            # state.log_queue.put((None, None, None, str(reward), None))
            # print_and_cr(f"[LOGGING] Stop logging")
            # state.is_logging_to = None
            terminate_logging(state)
            # state.handlers['p']('p', state) #hack
        return None
    if state.replay_counter == 0:
        print(state.robostate[:7] - state.replay_start_state)
        upper = state.CMD_DELTA_HIGH/5
        lower = state.CMD_DELTA_LOW/5
        # upper[:-1] = upper[:-1]/5
        # lower[:-1] = lower[:-1]/5
        if not np.all(np.isclose(state.robostate[:7], state.replay_start_state, atol=2e-1)):
        # if np.linalg.norm(state.robostate[:8] - state.replay_start_state) > 0.8:
            cmd = np.clip(state.replay_start_state,
                state.robostate[:7] + lower,
                state.robostate[:7] + upper)
            return cmd
        else:
            if not state.is_logging_to:
                state.handlers['l']('l', state) # force enter log
    cmd = state.replay_commands[state.replay_counter]
    cmd += np.random.standard_normal(size=cmd.shape) * 0.005 # Was .005
    if state.replay_delta:
        cmd += state.robostate[:7]
    state.replay_counter += 1
    return cmd
def load_pickle_commands(replay_filename, is_openloop):
    with open(replay_filename, 'rb') as f:
        paths = pickle.load(f)
    # sample_path_idx = np.random.choice(len(paths), size=1)[0]
    sample_path_idx = 0
    print_and_cr(f"[INFO] Sample replay path idx {sample_path_idx}")
    # sample_path = paths[sample_path_idx]
    sample_path = paths["demos"][sample_path_idx]
    commands = sample_path['commands']
    # import pdb; pdb.set_trace()
    # commands = commands[::2]
    return commands, sample_path['jointstates'][0, :7]