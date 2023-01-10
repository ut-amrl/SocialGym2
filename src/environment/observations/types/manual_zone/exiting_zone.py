import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment import ManualZoneUTMRSResponse


class ExitingZone(Observation):
    """
    Tracks if the agent is exiting the zone (exiting is defined as the agent overlapping with the right most line
    segment of the zone)
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def name(cls):
        return "exiting_zone"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: ManualZoneUTMRSResponse) -> np.array:
        return np.array([1. if env_response.exiting else 0.])
