import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict

GymObs = Union[Tuple, Dict, np.ndarray, int]


class TimeLimitWrapper(gym.Wrapper):
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

    def reset(self) -> GymObs:
        self.episode_steps = 0
        obs = self.env.reset()
        return obs

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, bool, Dict]:
        obs, reward, done, infos = self.env.step(action)
        self.episode_steps += 1

        if self.episode_steps == self.max_steps:
            done = True

        return obs, reward, done, infos
