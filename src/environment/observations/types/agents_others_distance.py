import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class AgentsOthersDistance(Observation):
    """
    Return the Normed distance between the agent and the other agents/humans
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
        return "agents_others_distance"

    def __len__(self):
        return self.num_others

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        distances = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans and len(env_response.human_poses) > 0:

            others_pose = np.concatenate(env_response.human_poses).reshape((-1, 3))
            others_distances = np.linalg.norm(others_pose[:, 0:2], axis=-1)
            distances[start_idx:len(others_distances)] = others_distances
            start_idx += len(others_distances)

        if self.allow_other_robots and len(env_response.other_robot_poses) > 0:
            robot_poses = np.concatenate(env_response.other_robot_poses).reshape((-1, 3))
            robot_distances = np.linalg.norm(robot_poses[:, 0:2], axis=-1)

            distances[start_idx:start_idx+len(robot_distances)] = robot_distances

        return distances

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        if self.allow_humans:
            self.num_others += max(env.ros_num_agents)
        if self.allow_other_robots:
            self.num_others += max(env.ros_num_agents) - 1
