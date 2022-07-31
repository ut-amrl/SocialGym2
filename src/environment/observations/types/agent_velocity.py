import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentVelocity(Observation):
    """
    TODO - warning, only works for single agent environments (1 robot) can be easily fixed if needed

    Returns the vx, vy of the agent in the current timestep
    """

    def __len__(self):
        # TODO - only works for single agent setups, update this if this changes
        return 3

    def observations(self, env: RosSocialEnv, env_response) -> np.array:
        agent_pose = poses_to_np_array(env_response.robot_vels)
        return agent_pose
