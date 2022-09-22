from abc import ABC, abstractmethod
import numpy as np
from typing import Dict

from src.environment.ros_social_gym import RosSocialEnv


class Reward(ABC):
    """
    Base class for rewarding or penalizing the agent in the simulation at each timestep according to the observation
    and environment configs.
    """

    weight: float

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        """
        :param weight: How much weight should this reward/penalty have
        """

        self.weight = weight

    @classmethod
    @abstractmethod
    def name(cls):
        """The name of the reward/penalty (used in other places, often to keep track of the types of rewards)"""
        raise NotImplemented("All rewards must have a name class method method.")

    @abstractmethod
    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        """See score documentation"""

        raise NotImplemented("All reward functions need a __score__ function.")

    def score(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        """
        Given the current environment instantiation, the current timesteps response from the environment, and the
        data_map object, calculate a float scalar as a reward.

        :param env: The RosSocialEnv class object currently being used in the sim
        :param env_response: The response from the env given the timestep
        :param observation_map: key observation pair from the Observer class.
        :returns: A weighted float scalar representing the amount of reward of the current timestep.
        """

        return self.weight * self.__score__(env, observation_map)

    def __call__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        """See score documentation"""

        # Shorthand for calling the score fn.
        return self.score(env, observation_map)

    def __setup__(self, env: RosSocialEnv):
        """
        Method that can setup the reward as needed before being used in the simulation.
        """

        return
