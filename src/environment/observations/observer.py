from typing import List, Dict, TYPE_CHECKING

import numpy as np

from src.environment.observations import Observation

if TYPE_CHECKING:
    from src.environment.ros_social_gym import RosSocialEnv


class Observer:

    registered_observations: List[Observation]

    def __init__(self, registered_observations: List[Observation]):
        self.registered_observations = registered_observations

    def __len__(self):
        return sum([len(x) for x in self.registered_observations])

    def make_observation_array(self, env: 'RosSocialEnv', env_response) -> np.array:
        return np.concatenate([x(env, env_response) for x in self.registered_observations])

    def make_observation_map(self, env: 'RosSocialEnv', env_response) -> Dict[str, np.array]:
        return {obs.name(): obs.observations(env, env_response) for obs in self.registered_observations}

    def setup(self, env: 'RosSocialEnv'):
        [x.__setup__(env) for x in self.registered_observations]
