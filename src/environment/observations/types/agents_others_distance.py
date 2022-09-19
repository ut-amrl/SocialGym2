import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


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

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        distances = np.zeros([len(self)])
        start_idx = 0

        if self.allow_humans:

            others_pose = poses_to_np_array(env_response.human_poses).reshape((-1, 3))
            others_distances = np.linalg.norm(others_pose[:, 0:2], axis=-1)
            distances[start_idx:len(others_distances)] = others_distances
            start_idx += len(others_distances)

        if self.allow_other_robots:
            robot_poses = poses_to_np_array(env_response.other_robot_poses).reshape((-1, 3))
            robot_distances = np.linalg.norm(robot_poses[:, 0:2], axis=-1)

            distances[start_idx:start_idx+len(robot_distances)] = robot_distances

        return distances

    def __setup__(self, env: RosSocialEnv):
        self.num_others = env.max_humans

        if hasattr(env, 'number_of_agents'):
            self.num_others += env.number_of_agents - 1