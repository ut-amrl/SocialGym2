import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class CollisionObservation(Observation):
    """
    Returns number of collisions
    """

    @classmethod
    def name(cls):
        return "collisions"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        return np.array([env_response.collision])
