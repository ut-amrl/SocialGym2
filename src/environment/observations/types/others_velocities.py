import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class OthersVelocities(Observation):
    """
    Returns the vx, vy, vtheta for each other agent/human in the environment.
    """

    num_others: int

    def __init__(
            self,
            allow_humans: bool = True,
            allow_other_robots: bool = True,
            ignore_theta: bool = False,
    ):
        super().__init__()

        self.num_others = 0

        self.allow_humans = allow_humans
        self.allow_other_robots = allow_other_robots
        self.ignore_theta = ignore_theta

    @classmethod
    def name(cls):
        return "others_velocities"

    def __len__(self):
        return self.num_others * (2 if self.ignore_theta else 3)

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        other_vels = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans and len(env_response.human_vels) > 0:
            human_vels = env_response.human_vels
            if self.ignore_theta:
                human_vels = [x[0:2] for x in human_vels]
            human_vels = np.concatenate(human_vels)
            other_vels[start_idx:len(human_vels)] = human_vels
            start_idx = start_idx + len(human_vels)
        if self.allow_other_robots and len(env_response.other_robot_vels) > 0:
            other_robot_vels = env_response.other_robot_vels
            if self.ignore_theta:
                other_robot_vels = [x[0:2] for x in other_robot_vels]
            other_robot_vels = np.concatenate(other_robot_vels)
            other_vels[start_idx:start_idx + len(other_robot_vels)] = other_robot_vels
            start_idx = start_idx + len(other_robot_vels)

        return other_vels

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
