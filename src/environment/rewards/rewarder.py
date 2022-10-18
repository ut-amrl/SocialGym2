from typing import List, Tuple, Dict, TYPE_CHECKING

import numpy as np
from copy import deepcopy

from src.environment.rewards import Reward

if TYPE_CHECKING:
    from src.environment.ros_social_gym import RosSocialEnv


class Rewarder:
    """
    Class that handles storing reward classes, calling them to sum their reward, and logging if a logger is given.
    """

    registered_rewards: List[List[Reward]]

    __step_idx__: int

    def __init__(
            self,
            registered_rewards: List[Reward],
    ):
        """
        :param registered_rewards: The reward class objects to run each timestep of the environment
        :param tbx_writer: TensorboardX Summary Writer used for logging
        """

        self.__orig_registered_rewards_stack__ = deepcopy(registered_rewards)
        self.registered_rewards = [registered_rewards]

    def reward(
            self,
            env: 'RosSocialEnv',
            observation_map,
    ) -> Tuple[List[np.array], List[Dict[str, float]]]:
        """
        Given the environments response for the current timestep, calculate the total reward (sum of all the registered
        reward class objects) and log the individual rewards as well as the total reward if the tensorboardX Summary
        Writer is present.

        :param env: The RosSocialEnv being used in the current episode
        :param observation_map: Dictionary storing all the observations of the environment
        """

        agent_rewards = [0. for _ in range(len(observation_map))]
        agent_reward_maps = [{} for _ in range(len(observation_map))]

        for i in range(len(observation_map)):
            agent_observations = observation_map[i]

            if i >= len(self.registered_rewards):
                new_stack = deepcopy(self.__orig_registered_rewards_stack__)
                [obs.__setup__(env) for obs in new_stack]
                self.registered_rewards.append(new_stack)

            total_reward = 0.
            reward_map = {}  # Build a map of reward names to reward values for the tensorboard logger.
            for reward_fn in self.registered_rewards[i]:
                reward = reward_fn(env, agent_observations)
                reward_map[reward_fn.name()] = reward
                total_reward += reward

            reward_map['total'] = total_reward

            agent_rewards[i] = total_reward
            agent_reward_maps[i] = reward_map

        return agent_rewards, agent_reward_maps

    def setup(self, env: 'RosSocialEnv'):
        """
        Helper for reward classes that require environment knowledge before being run.
        """

        [reward.__setup__(env) for stack in self.registered_rewards for reward in stack]

    def reset(self):
        [r.__reset__() for s in self.registered_rewards for r in s]
