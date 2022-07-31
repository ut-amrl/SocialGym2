from abc import ABC, abstractmethod

from src.environment.ros_social_gym import RosSocialEnv


class Reward(ABC):
    weight: float

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        self.weight = weight

    @abstractmethod
    def __score__(self, env: RosSocialEnv, env_response, data_map) -> float:
        """See score documentation"""

        raise NotImplemented("All reward functions need a __score__ function.")

    def score(self, env: RosSocialEnv, env_response, data_map) -> float:
        """
        Given the current environment instantiation, the current timesteps response from the environment, and the
        data_map object, calculate a float scalar as a reward.

        :param env: The RosSocialEnv class object currently being used in the sim
        :param env_response: The response from the env given the timestep
        :param data_map: TODO - what is this?
        :returns: A weighted float scalar representing the amount of reward of the current timestep.
        """

        return self.weight * self.__score__(env, env_response, data_map)

    def __call__(self, env: RosSocialEnv, env_response, data_map) -> float:
        """See score documentation"""

        # Shorthand for calling the score fn.
        return self.score(env, env_response, data_map)
