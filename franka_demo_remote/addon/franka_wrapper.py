import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env

import mujoco_py

class FrankaWrapper(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, "/home/franka/dev/franka_sim/franka_panda_connect.xml", frame_skip=5)

    def step(self, a):
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        reward = {'r':0}
        done = False
        return ob, reward, done, reward

    def get_fk(self, jointpos=None):
        if jointpos is not None:
            qpos = self.init_qpos
            qpos[:7] = jointpos[:7]
        else:
            qpos = self.sim.data.qpos
        qvel = self.init_qvel
        qvel[:] = 0
        self.set_state(qpos, qvel)
        return self.sim.data.get_site_xpos("end_effector")

    def reset_model(self, jointpos=None):
        qpos = self.init_qpos
        if jointpos is not None:
            qpos[:len(jointpos)] = jointpos
            qvel = self.init_qvel
        else:
            qvel = self.init_qvel + self.np_random.uniform(
                low=-0.005, high=0.005, size=self.model.nv
            )
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate(
            [
                self.sim.data.qpos.flat[:7],
                self.sim.data.qvel.flat[:7],
                self.sim.data.get_site_xpos("end_effector"),
            ]
        )

    def get_ik(self, target_ee, step_iter=1200):
        #self.reset_model()
        self.sim.data.mocap_pos[0] = target_ee
        for _ in range(step_iter):
            self.sim.step()
            #self.render() # Optionally turn on to see simulated EE control
        #print(f"{target_ee} -> {self.get_fk()}")
        return self.sim.data.qpos[:7]

if __name__ == "__main__":
    a = FrankaWrapper()
    print(a.get_fk([0,0,0,0,0,0,0]))
    print(a.get_fk([-0.145, -0.67, -0.052, -2.3, 0.145, 1.13, 0.029]))
    print(a.get_fk([0, -0.67, -0.052, -2.3, 0.145, 1.13, 0.029]))

    lower_bound = [0.2, -0.3, 0.4]
    upper_bound = [0.6, 0.3, 0.7]
    for _ in range(100):
        sample_ee = np.random.uniform(low=lower_bound, high=upper_bound)
        a.get_ik(sample_ee)
        print(f"{np.linalg.norm(sample_ee - a.get_fk())} \t {sample_ee} -> {a.get_fk()}")