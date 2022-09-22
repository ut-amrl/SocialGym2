import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class OthersPoses(Observation):
    """
    Returns the X, Y, Theta positions of the other agents/humans in the environment.
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
        return "others_poses"

    def __len__(self):
        return self.num_others * 3

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        other_poses = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans:
            human_poses = poses_to_np_array(env_response.human_poses)
            other_poses[start_idx:len(human_poses)] = human_poses
            start_idx = start_idx + len(human_poses)
        if self.allow_other_robots:
            robot_poses = poses_to_np_array(env_response.other_robot_poses)
            other_poses[start_idx:start_idx+len(robot_poses)] = robot_poses
            start_idx = start_idx + len(robot_poses)

        return other_poses

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
