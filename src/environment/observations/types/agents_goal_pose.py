import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class AgentsGoal(Observation):
    """
    Returns the X, Y, Theta positions of the goal of the agent in the environment.
    """

    def __init__(self, ignore_theta: bool = False):
        super().__init__()
        self.ignore_theta = ignore_theta

    @classmethod
    def name(cls):
        return "agents_goal"

    def __len__(self):
        return 2 if self.ignore_theta else 3

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        goal_pose = env_response.goal_pose[0:2]
        return goal_pose
