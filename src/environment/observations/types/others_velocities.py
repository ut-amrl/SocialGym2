import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class OthersVelocities(Observation):
    """
    Returns the vx, vy, vtheta for each other agent/human in the environment.
    """

    num_others: int

    def __init__(self):
        super().__init__()
        self.num_others = 0

    @classmethod
    def name(cls):
        return "others_velocities"

    def __len__(self):
        # TODO - only works for single agent setups, update this if this changes
        return self.num_others * 3

    def observations(self, env: RosSocialEnv, env_response) -> np.array:
        others_poses = poses_to_np_array(env_response.human_vels)
        return others_poses

    def __setup__(self, env: RosSocialEnv):
        self.num_others = env.max_humans
