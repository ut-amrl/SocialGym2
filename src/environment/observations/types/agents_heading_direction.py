import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class AgentsHeadingDirection(Observation):
    """
    Returns the heading direction of the agent ( via tan-1(vy / vx) )
    """

    @classmethod
    def name(cls):
        return "agents_heading_direction"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        agent_vels = env_response.robot_vels
        heading_direction = np.arctan(agent_vels[1] / agent_vels[0])
        return np.array([heading_direction])
