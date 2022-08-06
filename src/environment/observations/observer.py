from typing import List, Dict, TYPE_CHECKING

import numpy as np

from src.environment.observations import Observation

if TYPE_CHECKING:
    from src.environment.ros_social_gym import RosSocialEnv


class Observer:
    """
    Tracks all observations made in the environment
    """

    registered_observations: List[Observation]

    def __init__(self, registered_observations: List[Observation]):
        self.registered_observations = registered_observations

    def __len__(self):
        return sum([len(x) for x in self.registered_observations])

    def make_observation(self, env: 'RosSocialEnv', env_response) -> np.array:

        obs_map = {}
        obs_arr = []
        for obs in self.registered_observations:
            val = obs.observations(env, env_response)
            obs_map[obs.name()] = val
            obs_arr.append(val)

        return np.concatenate(obs_arr), obs_map

    def setup(self, env: 'RosSocialEnv'):
        [x.__setup__(env) for x in self.registered_observations]
