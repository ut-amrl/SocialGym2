import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWrapper
import pprint

GymObs = Union[Tuple, Dict, np.ndarray, int]


class TimeLimitWrapper(BaseParallelWrapper):
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
        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        obs, reward, done, truncs, infos = self.env.step(action)
        self.agents = self.unwrapped.agents

        self.episode_steps += 1

        if self.episode_steps >= self.max_steps:
            truncs = {k: True for k in truncs.keys()}
            done = {k: True for k in done.keys()}

        print(pprint.pformat(truncs), flush=True)
        # return obs, reward, done, truncs, infos
        return obs, reward, done, truncs, infos

    def seed(self, seed=None):
        pass
