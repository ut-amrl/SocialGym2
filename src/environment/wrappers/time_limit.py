import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWraper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class TimeLimitWrapper(BaseParallelWraper):
    """
    Limit the maximum number of steps per episode.

    :param env: Gym environment that will be wrapped
    :param max_steps: Max number of steps per episode
    """

    def __init__(self, env: gym.Env, max_steps: int = 2000):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)
        self.max_steps = max_steps
        self.episode_steps = 0

    def reset(self, seed=None, return_info=False, options=None):
        self.episode_steps = 0

        if not return_info:
            res = self.env.reset(seed=seed, options=options)
            self.agents = self.env.agents
            return res
        else:
            res, info = self.env.reset(
                seed=seed, return_info=return_info, options=options
            )
            self.agents = self.env.agents
            return res, info

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, bool, Dict]:
        obs, reward, done, infos = self.env.step(action)
        self.episode_steps += 1
        self.agents = self.env.agents

        if self.episode_steps == self.max_steps:
            done = {k: True for k in done.keys()}

        return obs, reward, done, infos

    def seed(self, seed=None):
        pass
