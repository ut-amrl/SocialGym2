import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class OthersVelocities(Observation):
    """
    Returns the vx, vy, vtheta for each other agent/human in the environment.
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
        return "others_velocities"

    def __len__(self):
        return self.num_others * 3

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        other_vels = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans:
            human_vels = poses_to_np_array(env_response.human_vels)
            other_vels[start_idx:len(human_vels)] = human_vels
            start_idx = start_idx + len(human_vels)
        if self.allow_other_robots:
            other_robot_vels = poses_to_np_array(env_response.other_robot_vels)
            other_vels[start_idx:start_idx + len(other_robot_vels)] = other_robot_vels
            start_idx = start_idx + len(other_robot_vels)

        return other_vels

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
