import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class OthersPoses(Observation):
    """
    Returns the X, Y, Theta positions of the other agents/humans in the environment.
    """

    num_others: int

    def __init__(self):
        super().__init__()
        self.num_others = 0

    @classmethod
    def name(cls):
        return "others_poses"

    def __len__(self):
        return self.num_others * 3

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        others_poses = poses_to_np_array(env_response.human_poses)
        return others_poses

    def __setup__(self, env: RosSocialEnv):
        self.num_others = env.max_humans
