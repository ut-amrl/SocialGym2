import numpy as np
import collections

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.utils import poses_to_np_array


class AgentsVelocity(Observation):
    """
    Returns the vx, vy of the agent in the current timestep
    """

    history_length: int
    history: collections.deque

    def __init__(self, history_length: int = 1):
        super().__init__()

        self.history_length = history_length
        # Ring buffer of max len, will remove oldest entry once the size limit has been reached.
        self.history = collections.deque(maxlen=len(self))

    @classmethod
    def name(cls):
        return "agents_velocity"

    def __len__(self):
        return self.history_length * 3

    def __observations__(self, env: RosSocialEnv, env_response) -> np.array:
        agent_pose = poses_to_np_array(env_response.robot_vels)

        # TODO - try extend here?
        [self.history.append(x) for x in agent_pose]

        # Make sure to left pad (i.e., [0, 0, 0, current value])
        return np.pad(np.array(self.history), pad_width=(len(self) - len(self.history), 0), mode='constant')
