import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class AgentsPreferredVelocity(Observation):
    """
    Returns the preferred velocity of the agent (really just a static number)
    """

    def __init__(self, preferred_velocity: float, *args, **kwargs):
        super().__init__(args, kwargs)
        self.preferred_velocity = preferred_velocity

    @classmethod
    def name(cls):
        return "agents_preferred_velocity"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        return np.array([self.preferred_velocity])
