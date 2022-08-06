import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict

from src.environment.ros_social_gym import RosSocialEnv


GymObs = Union[Tuple, Dict, np.ndarray, int]


class NewScenarioWrapper(Wrapper):

    new_scenario_episode_frequency: int
    episode_count: int
    env: RosSocialEnv

    def __init__(self, env: RosSocialEnv, new_scenario_episode_frequency: int = 5):
        super().__init__(env)
        self.new_scenario_episode_frequency = new_scenario_episode_frequency
        self.episode_count = 0

    def reset(self) -> GymObs:
        self.episode_count += 1

        if self.episode_count % self.new_scenario_episode_frequency == 0:
            self.env.new_scenario()

        obs = self.env.reset()
        return obs
