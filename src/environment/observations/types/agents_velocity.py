import numpy as np
import collections

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class AgentsVelocity(Observation):
    """
    Returns the vx, vy of the agent in the current timestep
    """

    history_length: int
    history: collections.deque
    ignore_theta: bool

    def __init__(self, history_length: int = 1, ignore_theta: bool = False):
        super().__init__()

        self.ignore_theta = ignore_theta

        self.history_length = history_length
        # Ring buffer of max len, will remove oldest entry once the size limit has been reached.
        self.history = collections.deque(maxlen=len(self))

    @classmethod
    def name(cls):
        return "agents_velocity"

    def __len__(self):
        return self.history_length * (2 if self.ignore_theta else 3)

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        agent_pose = env_response.robot_vels

        if self.ignore_theta:
            agent_pose = agent_pose[0:2]

        # TODO - try extend here?
        [self.history.append(x) for x in agent_pose]

        # Make sure to left pad (i.e., [0, 0, 0, current value])
        return np.pad(np.array(self.history), pad_width=(len(self) - len(self.history), 0), mode='constant')

    def __reset__(self):
        self.history = collections.deque(maxlen=len(self))
