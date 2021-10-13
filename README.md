# Franka Demo

Demo codebase for interacting with the Franka Robot Arm.

## Installation

- To communicate with Franka hardware please install [Polymetis](https://github.com/facebookresearch/fairo/tree/main/polymetis).
- To broadcast robot state we use Redis, `pip install redis`.
- Install this package: `pip install -e .`

## Usage

1. Launch the demo `python franka_demo/scripts/test_dummy_demo.py`.
2. Press corresponding keys to enter each mode. In dummy demo, try 's' to rotate the robot wrist by a small step.


**Teleoperation**

1. Ensure you installed Puppet (2020 Oct 13, we verified with [this version](https://github.com/vikashplus/puppet/tree/dac2c8cfe6a32259b36e355b4807ad7a2c060344))
2. Launch the demo `python franka_demo/scripts/launch_teleop_demo.py`
3. Launch puppet
3. Press 't' to the demo shell to enter teleop mode.

## Add More Functions

The core demo interfaces, `def run_demo` from `franka_demo.demo_interfaces`, launch two threads: a keyboard thread and a command thread.

1. The keyboard thread subscribes to your key press at 100Hz. You can write custom
handler to be invoked once a certain key is pressed. e.g. Press 'r' will put the robot on reset mode.

2. The command thread publishes the robot sensor information at 200Hz and send
command to the robot hardware at 40Hz. In order to send a command, it will check
which mode the robot is in and invoke the corresponding mode command generator.
For example, when the robot is in reset mode, its command generator resets the robot.
When the robot is in replay mode, its command generator parses a log file to commands and send the command in sequence.

3. To add a new function. Checkout `franka_demo/addon/teleop.py` to see how to
write your own key press handler and mode command generator. To launch the demo
with the new function, you can follow `franka_demo/scripts/launch_teleop_demo.py`.