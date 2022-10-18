import numpy as np
from abc import ABC, abstractmethod
from typing import List, Union

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.services import UTMRSResponse


class Observation(ABC):
    """
    Base class for an observation.
    """

    def __init__(self, *args, **kwargs):
        pass

    def observations(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        """
        Given the environment being used in the sim and the response from the current timestep, generate a numpy array
        that can be used as an observation for the Agent (subsequently passed into the policy network).

        Will pad the returned observation np array to the correct length specified in the __len__ to avoid shape
        mismatch in the observation space.

        :param env: The RosSocialEnv class object currently being used in the sim
        :param env_response: The response from the env given the timestep
        :returns: A numpy array that contains relevant observation data for the policy network.
        """

        if len(self) == 0:
            return np.array([])

        obs = self.__observations__(env, env_response)
        return np.nan_to_num(np.pad(obs, pad_width=(0,  len(self) - obs.shape[0]), mode='constant'))

    def __call__(self, env: RosSocialEnv, env_response: UTMRSResponse, *args, **kwargs) -> np.array:
        """
        See observations() documentation
        """

        return self.observations(env, env_response)

    def __setup__(self, env: RosSocialEnv):
        """
        Method that can setup the observation as needed before being used in the simulation.
        """

        return

    @abstractmethod
    def __len__(self) -> Union[int, List[int]]:
        """
        All observation classes should have the __len__ method return the expected dimensions of the numpy array
        """
        raise NotImplemented("All Observation classes need to have their __len__ attribute implemented.")

    @classmethod
    @abstractmethod
    def name(cls):
        raise NotImplemented("All observations must have a name attribute.")

    @abstractmethod
    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        """
        See observations() documentation
        """
        raise NotImplemented("All Observation classes need to have their __observations__() method defined.")

    def __reset__(self):
        return