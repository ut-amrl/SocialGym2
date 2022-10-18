from typing import List, Dict, Tuple, TYPE_CHECKING
import numpy as np
from copy import deepcopy

from src.environment.observations import Observation
from src.environment.services import UTMRSResponse

if TYPE_CHECKING:
    from src.environment.ros_social_gym import RosSocialEnv


class Observer:
    """
    Tracks all observations made in the environment
    """

    registered_observations: List[List[Observation]]

    def __init__(self, registered_observations: List[Observation]):
        self.__orig_registered_observation_stack__ = deepcopy(registered_observations)
        self.registered_observations = [registered_observations]

    def __len__(self):
        return sum([len(x) for x in self.registered_observations[0]])

    def make_observation(
            self,
            env: 'RosSocialEnv',
            env_response: List[UTMRSResponse]
    ) -> Tuple[List[np.array], List[Dict[str, any]]]:

        agent_observations = [np.zeros([len(self)]) for _ in range(len(env_response))]
        agent_obs_maps = [{} for _ in range(len(env_response))]

        for i in range(len(env_response)):
            agent_response = env_response[i]

            if i >= len(self.registered_observations):
                new_stack = deepcopy(self.__orig_registered_observation_stack__)
                [obs.__setup__(env) for obs in new_stack]
                self.registered_observations.append(new_stack)

            obs_map = {}
            obs_arr = []
            for obs in self.registered_observations[i]:
                val = obs.observations(env, agent_response)
                obs_map[obs.name()] = val
                obs_arr.append(val)

            agent_observations[i] = np.concatenate(obs_arr)
            agent_obs_maps[i] = obs_map

        return agent_observations, agent_obs_maps

    def setup(self, env: 'RosSocialEnv'):
        [obs.__setup__(env) for stack in self.registered_observations for obs in stack]

    def reset(self):
        [o.__reset__() for s in self.registered_observations for o in s]
