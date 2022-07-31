import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class OthersHeadingDirection(Observation):
    """
    Returns the heading direction of every other Agent/Human ( via tan-1(vx/vy) )
    """

    num_others: int

    def __init__(self):
        super().__init__()
        self.num_others = 0

    @classmethod
    def name(cls):
        return "others_heading_direction"

    def __len__(self):
        return self.num_others

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        others_poses = poses_to_np_array(env_response.human_vels).reshape((-1, 3))
        headings = np.arctanh(others_poses[:, 1] / others_poses[:, 0])

        return headings

    def __setup__(self, env: RosSocialEnv):
        self.num_others = env.max_humans
