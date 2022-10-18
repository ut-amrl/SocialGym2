import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment import ManualZoneUTMRSResponse


class AgentZonePriorityOrder(Observation):
    """
    Tracks the current order of the agent in the zone
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def name(cls):
        return "manual_zone_agent_zone_priority_order"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: ManualZoneUTMRSResponse) -> np.array:
        return np.array([env_response.agents_priority_order])
