import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentsPreferredVelocity(Observation):
    """
    TODO - warning, only works for single agent environments (1 robot) can be easily fixed if needed

    Returns the preferred velocity of the agent (really just a static number)
    """

    def __init__(self, preferred_velocity: float, *args, **kwargs):
        super().__init__(args, kwargs)
        self.preferred_velocity = preferred_velocity

    @classmethod
    def name(cls):
        return "agents_preferred_velocity"

    def __len__(self):
        # TODO - only works for single agent setups, update this if this changes
        return 1

    def observations(self, env: RosSocialEnv, env_response) -> np.array:
        return np.array([self.preferred_velocity])
