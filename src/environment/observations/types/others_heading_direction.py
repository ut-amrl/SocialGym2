import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils.utils import poses_to_np_array


class OthersHeadingDirection(Observation):
    """
    Returns the heading direction of every other Agent/Human ( via tan-1(vx/vy) )
    """

    num_others: int

    def __init__(
            self,
            allow_humans: bool = True,
            allow_other_robots: bool = True
    ):
        super().__init__()

        self.num_others = 0

        self.allow_humans = allow_humans
        self.allow_other_robots = allow_other_robots

    @classmethod
    def name(cls):
        return "others_heading_direction"

    def __len__(self):
        return self.num_others

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        other_heading_directions = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans and len(env_response.human_vels) > 0:
            others_poses = np.concatenate(env_response.human_vels).reshape((-1, 3))
            human_heading_directions = np.arctanh(others_poses[:, 1] / (others_poses[:, 0]))
            other_heading_directions[start_idx:len(human_heading_directions)] = human_heading_directions
            start_idx += len(human_heading_directions)
        if self.allow_other_robots and len(env_response.other_robot_vels) > 0:
            others_robot_poses = np.concatenate(env_response.other_robot_vels).reshape((-1, 3))
            robot_heading_directions = np.arctanh(others_robot_poses[:, 1] / (others_robot_poses[:, 0]))
            other_heading_directions[start_idx:start_idx+len(robot_heading_directions)] = robot_heading_directions

        return other_heading_directions

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
