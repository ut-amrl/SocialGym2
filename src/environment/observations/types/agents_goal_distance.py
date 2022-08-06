import numpy as np
import collections

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentsGoalDistance(Observation):
    """
    Returns the Norm of the agents pose and the goal pose
    """

    history_length: int
    history: collections.deque

    def __init__(self, history_length: int = 1):
        super().__init__()

        self.history_length = history_length
        # Ring buffer of max len, will remove oldest entry once the size limit has been reached.
        self.history = collections.deque(maxlen=history_length)

    @classmethod
    def name(cls):
        return "agents_goal_distance"

    def __len__(self):
        return self.history_length

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        agent_pose = poses_to_np_array(env_response.robot_poses)[0:2]
        goal_pose = poses_to_np_array(env_response.goal_pose)[0:2]
        goal_distance = np.linalg.norm(agent_pose - goal_pose)

        self.history.append(goal_distance)

        # Make sure to left pad (i.e., [0, 0, 0, current value])
        return np.pad(np.array(self.history), pad_width=(len(self) - len(self.history), 0), mode='constant')
