from franka_demo.hardware_franka import get_args
from franka_demo.demo_interfaces import run_demo
from franka_demo.addon import add_logging_function

def _press_open_gripper(key_pressed, state):
    state.franka.open_gripper()
    print("Opened gripper")

def _press_close_gripper(key_pressed, state):
    state.franka.close_gripper()
    print("Closed gripper")

def callback_func(state):
    state.handlers['['] = _press_open_gripper
    state.handlers[']'] = _press_close_gripper
    add_logging_function(state)

if __name__ == "__main__":

    args = get_args()

    run_demo(callback_func, params={
        'ip_address': args.server_ip,
        'gripper': True,
    })