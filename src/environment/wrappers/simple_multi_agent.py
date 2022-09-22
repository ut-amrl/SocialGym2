from copy import deepcopy
import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from stable_baselines3.common.base_class import BaseAlgorithm

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations.observer import Observer

GymObs = Union[Tuple, Dict, np.ndarray, int]


class SimpleMultiAgent(Wrapper):

    new_scenario_episode_frequency: int
    episode_count: int
    env: RosSocialEnv

    def __init__(self, env: RosSocialEnv, model: BaseAlgorithm, number_of_agents: Union[int, Tuple[int, int]] = 3):
        super().__init__(env)
        self.number_of_agents = number_of_agents
        self.episode_count = 0

        self.env.new_scenario = self.new_scenario
        self.env.default_action = self.default_action
        # self.env.MakeObservation = self.make_observation
        self.env.number_of_agents = self.number_of_agents
        # self.env.CalculateReward = self.calcualte_reward

        self.model = model

    def default_action(self):
        return [0]*self.number_of_agents

    def calcualte_reward(self, res, obs_map, dataMap):
        return self.env.rewarder.reward(self.env, obs_map, dataMap) + \
               sum([
                   x.reward(self.env, obs_map, dataMap) for x, obs_map in zip(self.other_rewarders, self.other_obs_maps)
               ])

    def initialize(self):
        self.env.initialize()

        # self.other_observers = [deepcopy(self.env.observer) for _ in range(self.number_of_agents - 1)]
        # self.other_rewarders = [deepcopy(self.env.rewarder) for _ in range(self.number_of_agents - 1)]

        self.other_obs_maps = [None] * (self.number_of_agents - 1)
        self.other_actions = [0] * (self.number_of_agents - 1)

    def new_scenario(self):
        self.env.scenario.generate_scenario(num_humans=self.env.num_humans, num_agents=self.number_of_agents)

    def make_observation(self, res):
        main_observation = self.env.observer.make_observation(self.env, res.robot_responses[0])

        for i in range(len(res.robot_responses) - 1):
            self.other_obs_maps[i] = self.other_observers[i].make_observation(self.env, res.robot_responses[i+1])
            if self.model:
                action, _ = self.model.predict(self.other_obs_maps[i][0], deterministic=True)
                self.other_actions[i] = action
            else:
                self.other_actions[i] = 0

        return main_observation

    def step(self, action):
        actions = [action, *self.other_actions]
        return self.env.step(actions)
