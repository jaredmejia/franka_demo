# ------------------------------------------------------------------------------
# Dummy robot interface for test, no need to launch Polymetis
# ------------------------------------------------------------------------------

import numpy as np

class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class DummyPolymetisRobot():
    def __init__(self):
        self.response = dotdict({'end':-1})

    def get_previous_interval(self):
        return self.response

class DummyFrankaArm():
    def __init__(self, name, ip_address, **kwargs):
        self.robot = DummyPolymetisRobot()

    def default_policy(self, kq_ratio=5, kqd_ratio=5):
        pass

    def connect(self, policy=None):
        pass

    def okay(self):
        pass

    def close(self):
        pass

    def reset(self):
        pass

    def get_sensors(self):
        return np.arange(7, dtype=np.float32)

    def get_sensors_offsets(self):
        return self.get_sensors()

    def apply_commands(self, cmd):
        pass

    def apply_commands_offsets(self, cmd):
        pass

