import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWraper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class CollisionEpisodeEnder(BaseParallelWraper):
    """
    You collide you die!

    :param env: Gym environment that will be wrapped
    """

    def __init__(self, env: gym.Env):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, bool, Dict]:
        obs, reward, done, infos = self.env.step(action)
        self.agents = self.env.agents

        collision = any([m['collisions'] == 1 for m in self.unwrapped.last_obs_maps])

        if collision:
            done = {k: True for k in done.keys()}

        return obs, reward, done, infos

    def seed(self, seed=None):
        pass
