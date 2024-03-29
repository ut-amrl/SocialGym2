import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class OthersPoses(Observation):
    """
    Returns the X, Y, Theta positions of the other agents/humans in the environment.
    """

    num_others: int

    def __init__(
            self,
            allow_humans: bool = True,
            allow_other_robots: bool = True,
            actuals: bool = False,
            ignore_theta: bool = False,
    ):
        super().__init__()

        self.num_others = 0

        self.allow_humans = allow_humans
        self.allow_other_robots = allow_other_robots
        self.actuals = actuals
        self.ignore_theta = ignore_theta

    @classmethod
    def name(cls):
        return "others_poses"

    def __len__(self):
        return self.num_others * (2 if self.ignore_theta else 3)

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        other_poses = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans and len(env_response.human_poses) > 0:
            human_poses = env_response.human_poses
            if self.ignore_theta:
                human_poses = [x[0:2] for x in human_poses]
            human_poses = np.concatenate(human_poses)
            other_poses[start_idx:len(human_poses)] = human_poses
            start_idx = start_idx + len(human_poses)
        if self.allow_other_robots and len(env_response.other_robot_poses) > 0:
            other_robot_poses = env_response.other_robot_poses
            if self.ignore_theta:
                other_robot_poses = [x[0:2] for x in other_robot_poses]
            robot_poses = np.concatenate(other_robot_poses)
            if self.actuals:
                robot_poses += np.concatenate([env_response.robot_poses] * (max(env.ros_num_agents) - 1))
            other_poses[start_idx:start_idx+len(robot_poses)] = robot_poses
            start_idx = start_idx + len(robot_poses)

        return other_poses

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
