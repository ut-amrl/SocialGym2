import numpy as np
from typing import Dict

from src.environment.observations import AgentsVelocity
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class VelocityControl(Reward):
    """
    Penalty for punishing abrupt changes in velocity
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "velocity_control"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        assert AgentsVelocity.name() in observation_map, \
            'Velocity Control requires the agents velocity to be in the observation'
        assert len(observation_map[AgentsVelocity.name()]) >= 6, \
            'Velocity Control requires that the AgentsVelocity has at least a history length of 2.'

        vels = observation_map[AgentsVelocity.name()]
        change = (vels[0:2] - vels[3:5]).sum() ** 2

        return -change
