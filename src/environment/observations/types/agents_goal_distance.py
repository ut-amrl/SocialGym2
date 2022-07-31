import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentsGoalDistance(Observation):
    """
    Returns the Norm of the agents pose and the goal pose
    """

    @classmethod
    def name(cls):
        return "agents_goal_distance"

    def __len__(self):
        return 1

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        agent_pose = poses_to_np_array(env_response.robot_poses)
        goal_pose = poses_to_np_array(env_response.goal_pose)

        goal_distance = np.linalg.norm(agent_pose - goal_pose)
        return np.array([goal_distance])
