import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentsVelocity(Observation):
    """
    Returns the vx, vy of the agent in the current timestep
    """

    @classmethod
    def name(cls):
        return "agents_velocity"

    def __len__(self):
        return 3

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        agent_pose = poses_to_np_array(env_response.robot_vels)
        return agent_pose
