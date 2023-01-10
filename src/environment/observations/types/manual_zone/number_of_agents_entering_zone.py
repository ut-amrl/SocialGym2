import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment import ManualZoneUTMRSResponse


class NumberOfAgentsEnteringZone(Observation):
    """
    Tracks the number of agents entering the zone (entering is defined as the number of agents intersecting the
    left most line segment in the zone)
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def name(cls):
        return "number_of_agents_entering_zone"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: ManualZoneUTMRSResponse) -> np.array:
        return np.array([env_response.number_agents_entering])
