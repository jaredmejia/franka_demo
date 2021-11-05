from franka_demo_remote.hardware_franka import get_args
from franka_demo_remote.demo_interfaces import run_demo

def _press_open_gripper(key_pressed, state):
    #state.franka.gripper.goto(width=0.2, speed=0.1, force=1.0)
    state.franka.open_gripper()
    print("Opened gripper")

def _press_close_gripper(key_pressed, state):
    #state.franka.gripper.grasp(speed=0.1, force=1.0)
    state.franka.close_gripper()
    print("Closed gripper")

def callback_func(state):
    state.handlers['['] = _press_open_gripper
    state.handlers[']'] = _press_close_gripper

if __name__ == "__main__":

    args = get_args()

    run_demo(callback_func, params={
        'ip_address': args.server_ip,
    })