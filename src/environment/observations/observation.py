import numpy as np
from abc import ABC, abstractmethod
from typing import List, Union

from src.environment.ros_social_gym import RosSocialEnv


class Observation:

    def __init__(self, *args, **kwargs):
        pass

    @abstractmethod
    def observations(self, env: RosSocialEnv, env_response) -> np.array:
        """
        Given the environment being used in the sim and the response from the current timestep, generate a numpy array
        that can be used as an observation for the Agent (subsequently passed into the policy network).

        :param env: The RosSocialEnv class object currently being used in the sim
        :param env_response: The response from the env given the timestep
        :returns: A numpy array that contains relevant observation data for the policy network.
        """

        raise NotImplemented("All Observation classes need to have their observations() method defined.")

    @abstractmethod
    def __len__(self) -> Union[int, List[int]]:
        """
        All observation classes should have the __len__ method return the expected dimensions of the numpy array
        """
        raise NotImplemented("All Observation classes need to have their __len__ attribute implemented.")

    def __call__(self, env: RosSocialEnv, env_response, *args, **kwargs) -> np.array:
        return self.observations(env, env_response)