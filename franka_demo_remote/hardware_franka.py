# ==============================================================================
# Copy of hardware_base.py and hardware_franka.py from mj_envs/robot
# Any modifications should be synced to the original repo
# ==============================================================================
# hardware_base.py
# ------------------------------------------------------------------------------
# Base robot class for other hardware devices to inheret from
import abc
from random import random

class hardwareBase(abc.ABC):
    def __init__(self, name, *args, **kwargs):
        self.name = name

    @abc.abstractmethod
    def connect(self):
        """Establish hardware connection"""

    @abc.abstractmethod
    def okay(self):
        """Return hardware health"""

    @abc.abstractmethod
    def close(self):
        """Close hardware connection"""

    @abc.abstractmethod
    def reset(self):
        """Reset hardware"""

    @abc.abstractmethod
    def get_sensors(self):
        """Get hardware sensors"""

    @abc.abstractmethod
    def apply_commands(self):
        """Apply hardware commands"""

    def __del__(self):
        self.close()

# ------------------------------------------------------------------------------
# hardware_franka.py
# ------------------------------------------------------------------------------

from typing import Dict, Sized
import time
import sys

import numpy as np
from numpy.core.fromnumeric import size
import torch

from polymetis import RobotInterface
import torchcontrol as toco
import argparse

class JointPDPolicy(toco.PolicyModule):
    """
    Custom policy that performs PD control around a desired joint position
    """

    def __init__(self, desired_joint_pos, kq, kqd, **kwargs):
        """
        Args:black
            desired_joint_pos (int):    Number of steps policy should execute
            hz (double):                Frequency of controller
            kq, kqd (torch.Tensor):     PD gains (1d array)
        """
        super().__init__(**kwargs)

        self.q_desired = torch.nn.Parameter(desired_joint_pos)

        # Initialize modules
        self.feedback = toco.modules.JointSpacePD(kq, kqd)

    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # Parse states
        q_current = state_dict["joint_positions"]
        qd_current = state_dict["joint_velocities"]

        # Execute PD control
        output = self.feedback(
            q_current, qd_current, self.q_desired, torch.zeros_like(qd_current)
        )

        return {"joint_torques": output}

class FrankaArm():
    CMD_SHAPE = 7
    # Notice that the start position will add joint offset before send to hdware
    START_POSITION = np.array([0.1828, -0.4909, -0.0093, -2.4412, 0.2554, 0.3310, 0.0])
    # START_POSITION = np.array([0.1828, -0.4909, -0.0093, -2.4412, 0.2554, 0.3310, -1.0])

    # Close left [ 0.19719838, -0.31133446,  0.63921565, -2.27884555,  0.26551878, 0.49570596,  0.58121759]
    # Close righ [-0.3159813 , -0.38090691, -0.44509837, -2.32920551, -0.72878891, 0.57118762, -1.82918906]
    # Far ottom  [0.24321331,  0.426597  ,  0.30724603, -1.75327575, -0.09180571, 0.78220975, -1.18756342]
    # Fright top [-0.25507489,  0.13030759, -0.27166939, -1.68428528, -0.64755744, 0.55247104, -0.58670473
    # np.array(  [0.1828, -0.4909, -0.0093, -2.4412, 0.2554, 0.3310, 0.0]
    # np.array(  [-0.145, -0.67, -0.052, -2.3, 0.145, 1.13, 0.0])
    # Notice that the joint limit is applied after applying joint offset
    JOINT_LIMIT_MIN = np.array(
        [-2.8773, -1.7428, -2.8773, -3.0018, -2.8773, 0.0025, -2.8773])
    JOINT_LIMIT_MAX = np.array(
        [2.8773, 1.7428, 2.8773, -0.1398, 2.8773, 3.7325, 2.8773])

    def __init__(self, name, ip_address, **kwargs):
        self.name = name
        self.robot = None
        self.JOINT_OFFSET = np.array(
            [0, 0, 0, 0, 0., np.pi/2, np.pi/4],
            dtype=np.float32) # TODO replace hardcode with config

        # Initialize self.robot interface
        self.robot = RobotInterface(
            ip_address=ip_address,
            enforce_version=False,
        )

    def default_policy(self, kq_ratio=1.5, kqd_ratio=1.5):
        q_initial = self.get_sensors().clone()
        kq = kq_ratio * torch.Tensor(self.robot.metadata.default_Kq)
        kqd = kqd_ratio * torch.Tensor(self.robot.metadata.default_Kqd)
        return JointPDPolicy(
                desired_joint_pos=torch.tensor(q_initial),
                kq=kq, kqd=kqd,
        )

    def connect(self, policy=None):
        """Establish hardware connection"""
        if policy==None:
            print("\nRunning PD policy...")
            policy = self.default_policy()
        self.robot.send_torch_policy(policy, blocking=False)

    def okay(self):
        """Return hardware health"""
        if self.robot:
            return True
        else:
            return False

    def close(self):
        """Close hardware connection"""
        print("Terminating PD policy...")
        state_log = True
        if self.robot:
            self.reset()
            state_log = self.robot.terminate_current_policy()
        return state_log

    def reset(self):
        """Reset hardware"""
        #self.robot.set_home_pose(torch.tensor(random_pose))
        self.robot.go_home()

    def get_sensors(self):
        """Get hardware sensors"""
        return self.robot.get_joint_positions()

    def apply_commands(self, q_desired):
        """Apply hardware commands"""
        q_des_tensor = torch.tensor(q_desired)
        self.robot.update_current_policy({"q_desired": q_des_tensor})

    def get_sensors_offsets(self):
        """Get hardware sensors (apply offset)"""
        joint_angle = self.robot.get_joint_positions()
        return joint_angle - self.JOINT_OFFSET

    def apply_commands_offsets(self, q_desired):
        """Apply hardware commands (apply offset)"""
        #if np.any(np.logical_or(q_desired < self.JOINT_LIMIT_LOW, q_desired > self.JOINT_LIMIT_HIGH)):
        #    sys.stdout.write(f"Command outside joint limit {q_desired}\r\n")
        #    np.clip(q_desired,
        #        self.JOINT_LIMIT_LOW,
        #        self.JOINT_LIMIT_HIGH,
        #        out=q_desired)
        q_des_tensor = np.array(q_desired) + self.JOINT_OFFSET
        q_des_tensor = torch.tensor(np.clip(
            q_des_tensor, self.JOINT_LIMIT_MIN, self.JOINT_LIMIT_MAX))
        #print('RAW_CMD_DIFF', q_des_tensor.float() - self.robot.get_joint_positions())
        self.robot.update_current_policy({"q_desired": q_des_tensor.float()})

    def __del__(self):
        self.close()

from franka_demo_remote.hardware_gripper import Gripper

class FrankaArmWithGripper(FrankaArm):
    CMD_SHAPE = 8
    STATE_SHAPE = 8
    START_POSITION = np.array(list(FrankaArm.START_POSITION) + [0.08], dtype=np.float32)

    def __init__(self, name, ip_address, **kwargs):
        super(FrankaArmWithGripper, self).__init__(name, ip_address, **kwargs)
        self.gripper = Gripper(ip_address)
        self.state_nparray = np.zeros(self.STATE_SHAPE)

    def reset(self):
        self.open_gripper()
        super(FrankaArmWithGripper, self).reset()

    def close_gripper(self):
        self.gripper.goto(width=0.0, speed=0.1, force=0.1)

    def open_gripper(self):
        self.gripper.goto(width=self.gripper.gripper_max_width, speed=0.1, force=0.1)

    def get_sensors_offsets(self):
        """Get hardware sensors (apply offset)"""
        self.state_nparray[0:7] = super(FrankaArmWithGripper, self).get_sensors_offsets()
        self.state_nparray[7] = self.gripper.get()
        return self.state_nparray

    def apply_commands_offsets(self, q_desired):
        """Apply hardware commands (apply offset)"""
        super(FrankaArmWithGripper, self).apply_commands_offsets(q_desired[:-1])
        self.gripper.goto(width=q_desired[-1], speed=0.1, force=0.1)

class FrankaArmWithRobotiQGripper(FrankaArmWithGripper):

    def __init__(self, name, ip_address, **kwargs):
        super(FrankaArmWithRobotiQGripper, self).__init__(name, ip_address, **kwargs)
        self.JOINT_OFFSET[6] -= np.pi/4

class FrankaArmIncludeJointVel(FrankaArm):

    STATE_SHAPE = 14 # 7 jointpos, 7 joint vel

    def __init__(self, name, ip_address, **kwargs):
        super(FrankaArmIncludeJointVel, self).__init__(name, ip_address, **kwargs)
        self.state_nparray = np.zeros(self.STATE_SHAPE)

    # NOTE: don't define get_sensors, it is meant to ONLY return pos information
    # so we can initialize the default policy correctly.

    def get_sensors_offsets(self):
        """Get hardware sensors (apply offset)"""
        # super(FrankaArmIncludeJointVel, self).get_sensors_offsets()
        self.state_nparray[0:7] = super(FrankaArmIncludeJointVel, self).get_sensors_offsets()
        self.state_nparray[7:14] = self.robot.get_joint_velocities()
        return self.state_nparray

# Get inputs from user
def get_args():
    parser = argparse.ArgumentParser(description="OptiTrack Client: Connects to \
        the server and fetches streaming data")

    parser.add_argument("-i", "--server_ip",
                        type=str,
                        help="IP address or hostname of the franka server",
                        default="172.16.0.1") # 10.0.0.123 # "169.254.163.91",
    parser.add_argument("-r", "--remote",
                action='store_true') 
    return parser.parse_args()


if __name__ == "__main__":

    args = get_args()

    # user inputs
    time_to_go = 2*np.pi
    joint = 6
    m = 0.5  # magnitude of sine wave (rad)
    T = 2.0  # period of sine wave
    hz = 50  # update frequency

    # Initialize robot
    franka = FrankaArm(name="Franka-Demo", ip_address=args.server_ip)

    import time
    start = time.time()
    franka.robot.go_home()
    # connect to robot with default policy
    franka.connect(policy=None)

    print(f"Took {time.time() - start} sec to go home and set policy")

    q_initial = franka.get_sensors_offsets()
    # q_desired = np.array(q_initial, dtype=np.float64)
    q_desired = np.zeros(franka.CMD_SHAPE)
    q_desired[:] = q_initial[:len(q_desired)]
    print(f"Shape of q_desired: {list(q_desired.shape)}")
    print(f"Starting sine motion updates... will repeat for {time_to_go} seconds")

    for i in range(int(time_to_go * hz)):
        q_desired[joint] = q_initial[joint] + m * np.sin(np.pi * i / (T * hz))
        franka.apply_commands_offsets(q_desired = q_desired)
        time.sleep(1 / hz)

    print("Finished moving")
    franka.close()