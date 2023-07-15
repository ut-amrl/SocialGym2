import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWrapper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class RewardStripper(BaseParallelWrapper):
    """
    When an agent succeeds, remove the

    :param env: Gym environment that will be wrapped
    """

    def __init__(self, env: gym.Env):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)

        self.rws_successes = []
        self.agents = self.unwrapped.agents

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        obs, rewards, done, infos = self.env.step(action)
        self.agents = self.unwrapped.agents

        if len(self.rws_successes) == 0:
            self.rws_successes = [False] * len(self.unwrapped.last_obs_maps)

        agent_rewards = {
            k: v if not self.rws_successes[idx] else 0. for idx, (k, v) in enumerate(rewards.items())
        }

        for idx, _obs in enumerate(self.unwrapped.last_obs_maps):
            if _obs['success_observation'] == 1:
                self.rws_successes[idx] = True

        # return obs, reward, done, truncs, infos
        return obs, agent_rewards, done, infos

    def seed(self, seed=None):
        pass
