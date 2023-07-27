import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWrapper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class CollisionEpisodeEnder(BaseParallelWrapper):
    """
    You collide you die!

    :param env: Gym environment that will be wrapped
    """

    def __init__(self, env: gym.Env):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)
        self.agents = self.unwrapped.agents

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        obs, reward, done, truncs, infos = self.env.step(action)
        self.agents = self.unwrapped.agents

        collision = any([m['collisions'] == 1 for m in self.unwrapped.last_obs_maps])

        if collision:
            # truncs = {k: True for k in done.keys()}
            done = {k: True for k in done.keys()}
            truncs = {k: True for k in truncs.keys()}

        # return obs, reward, done, truncs, infos
        return obs, reward, done, truncs, infos

    def seed(self, seed=None):
        pass
