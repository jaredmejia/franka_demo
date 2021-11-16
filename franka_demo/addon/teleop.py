import numpy as np

from franka_demo.utils import print_and_cr
from franka_demo.demo_interfaces import redis_send_dummy_command, CMD_DTYPE

def _press_teleop(key_pressed, state):
    print_and_cr(f"[INFO] Enter Teleop")
    robostate = np.array(state.franka.get_sensors_offsets(), dtype=CMD_DTYPE)
    redis_send_dummy_command(state.redis_store, robostate) # TODO pick out pos
    state.mode = 'teleop'

def __cmd_teleop(state, timestamp):
    joint_pos_desired = state.redis_receive_command()
    #print_and_cr(f"Received joint command {joint_pos_desired}")
    #print_and_cr(f"Current robot state {state.robostate}")
    #print_and_cr(f"Diff {joint_pos_desired - state.robostate}")
    np.clip(joint_pos_desired,
        state.robostate + state.CMD_DELTA_LOW,
        state.robostate + state.CMD_DELTA_HIGH,
        out=joint_pos_desired)
    #print_and_cr(f"Clipped cmd's delta {joint_pos_desired - state.robostate}")
    return joint_pos_desired

def add_teleop_function(state):
    state.handlers['t'] = _press_teleop
    state.modes['teleop'] = __cmd_teleop