import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class SuccessObservation(Observation):
    """
    Returns if there was a success in the episode
    """

    @classmethod
    def name(cls):
        return "success_observation"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        return np.array([1.0 if env_response.success else 0.0])
