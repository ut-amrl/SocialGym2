import gym
from gym import Wrapper, Env
from pettingzoo.utils.wrappers import BaseParallelWraper
import numpy as np
from typing import Union, Tuple, Dict

from src.environment.ros_social_gym import RosSocialEnv


GymObs = Union[Tuple, Dict, np.ndarray, int]


class NewScenarioWrapper(BaseParallelWraper):

    new_scenario_episode_frequency: int
    episode_count: int
    env: RosSocialEnv

    def __init__(self, env: Union[RosSocialEnv, gym.Env], new_scenario_episode_frequency: int = 5):
        super().__init__(env)
        self.new_scenario_episode_frequency = new_scenario_episode_frequency
        self.episode_count = 0

    def reset(self, seed=None, return_info=False, options=None):
        self.episode_count += 1

        if self.episode_count % self.new_scenario_episode_frequency == 0:
            self.unwrapped.new_scenario()

        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def seed(self, seed=None):
        pass
