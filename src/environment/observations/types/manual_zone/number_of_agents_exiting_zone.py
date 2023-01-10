import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment import ManualZoneUTMRSResponse


class NumberOfAgentsExitingZone(Observation):
    """
    Tracks the number of agents exiting the zone (exiting is defined as the agent overlapping with the right most
    line segment of the zone)
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def name(cls):
        return "number_of_agents_exiting_zone"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: ManualZoneUTMRSResponse) -> np.array:
        return np.array([env_response.number_agents_exiting])
