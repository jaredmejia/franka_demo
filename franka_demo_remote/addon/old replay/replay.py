import os
import numpy as np

from franka_demo_remote.utils import print_and_cr
from franka_demo_remote.demo_interfaces import CMD_DTYPE

def add_replay_function(state):
	state.handlers['p'] = _press_replay
	state.modes['replay'] = __cmd_replay

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
		state.replay_commands = load_csv_commands(replay_filename)
		state.replay_commands_length = len(state.replay_commands)
	except Exception as e:
		print_and_cr(f"[ERROR] (in replay.py) Failed to load replay file at {replay_filename}")
		print(e)
		return

	state.replay_counter = 0
	print_and_cr(f"[INFO] Replaying commands from {replay_filename}")
	state.mode = 'replay'

def __cmd_replay(state, timestamp):
	# TODO: call move / planning to move to the beginning of the record
	if state.replay_counter == state.replay_commands_length:
		return None
	cmd = state.replay_commands[state.replay_counter]
	state.replay_counter += 1
	return cmd

def load_csv_commands(fn):
	with open(fn, "r") as file_handler:
		records = file_handler.readlines()
	raw_commands = [line.split(',')[2] for line in records]
	commands = [
		None if cmd == 'None' else np.fromstring(cmd, dtype=CMD_DTYPE, sep=' ') for cmd in raw_commands
	]
	return commands
