# Franka Demo

Demo codebase for interacting with the Franka Robot Arm.

## Installation

- To communicate with Franka hardware please install [Polymetis](https://github.com/facebookresearch/fairo/tree/main/polymetis). We used the latest Polymetis from facebook research, as of 2021 Oct.
- To broadcast robot state we use Redis. Install redis on your computer. Then install Redis python client: `pip install redis`.
- Install this package: `pip install -e .`

## Usage

1. Turn on your robot. Launch Polyemtis server on NUC machine.
2. Launch the demo `python franka_demo_remote/scripts/test_dummy_demo.py`.
3. Wait for the demo to communicate with the robot.
4. Then you can press 'h' for help.
5. Press corresponding keys to enter each mode. In dummy demo, try 's' to rotate the robot wrist by a small step.


**Teleoperation**

1. Ensure you installed Puppet (2020 Oct 13, we verified with [this version](https://github.com/vikashplus/puppet/tree/dac2c8cfe6a32259b36e355b4807ad7a2c060344))
2. Launch the demo `python franka_demo_remote/scripts/launch_teleop_demo.py`
3. Launch puppet
4. Press 't' to the demo shell to enter teleop mode.
5. Press 'l' to log data. Press again to finish logging.

## Add More Functions

The core demo interfaces, `run_demo` from `franka_demo_remote.demo_interfaces`, launch two threads: a keyboard thread and a command thread.

1. The keyboard thread subscribes to your key press at 100Hz. You can write custom
handler to be invoked once a certain key is pressed. e.g. Press 'r' will put the robot on reset mode.

2. The command thread publishes the robot sensor information at 200Hz and send
command to the robot hardware at 40Hz. In order to send a command, it will check
which mode the robot is in and invoke the corresponding mode command generator.
For example, when the robot is in reset mode, its command generator resets the robot.
When the robot is in replay mode, its command generator parses a log file and send the logged command in sequence.

3. To add a new function. Checkout `franka_demo_remote/addon/teleop.py` to see how to
write your own key press handler and mode command generator. To launch the demo
with the new function, you can follow `franka_demo_remote/scripts/launch_teleop_demo.py`.