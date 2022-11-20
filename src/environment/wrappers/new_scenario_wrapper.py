import gym
from gym import Wrapper, Env
from pettingzoo.utils.wrappers import BaseParallelWraper
import numpy as np
from typing import Union, Tuple, Dict, List

from src.environment.ros_social_gym import RosSocialEnv


GymObs = Union[Tuple, Dict, np.ndarray, int]


class NewScenarioWrapper(BaseParallelWraper):

    new_scenario_episode_frequency: int
    episode_count: int
    env: RosSocialEnv
    plans: List[Tuple[int, int]]


    def __init__(self, env: Union[RosSocialEnv, gym.Env], new_scenario_episode_frequency: int = 5, plans: List[Tuple[int, int]] = ()):
        super().__init__(env)
        self.new_scenario_episode_frequency = new_scenario_episode_frequency
        self.episode_count = 0
        self.eval_episode_count = 0
        self.plans = list(sorted(plans, key=lambda x: x[0], reverse=True))


    def reset(self, seed=None, return_info=False, options=None):
        in_eval = self.unwrapped.in_eval
        new_episode = False

        if in_eval:
            self.eval_episode_count += 1
            new_episode = self.eval_episode_count % self.new_scenario_episode_frequency == 0
        else:
            self.episode_count += 1
            new_episode = self.episode_count % self.new_scenario_episode_frequency == 0

        if new_episode:
            if len(self.plans) == 0 or in_eval:
                self.unwrapped.new_scenario()
            else:
                for e, a in self.plans:
                    if e <= self.episode_count:
                        self.unwrapped.new_scenario(num_agents=a)
                        self.unwrapped.curr_num_agents = a
                        break

        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def seed(self, seed=None):
        pass
