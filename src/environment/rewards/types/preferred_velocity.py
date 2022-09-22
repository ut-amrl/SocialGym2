import numpy as np
from typing import Dict

from src.environment.observations import AgentsVelocity, AgentsPreferredVelocity
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class PreferredVelocity(Reward):
    """
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "preferred_velocity"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        assert AgentsVelocity.name() in observation_map, \
            'Velocity Control requires the agents velocity to be in the observation'
        assert len(observation_map[AgentsVelocity.name()]) >= 6, \
            'Velocity Control requires that the AgentsVelocity has at least a history length of 2.'
        assert AgentsPreferredVelocity.name() in observation_map, \
            'Preferred Velocity requires that the AgentsPreferredVelocity is in the observation'

        vels = observation_map[AgentsVelocity.name()]
        vel = np.sqrt((vels[0:2]).sum() ** 2)

        diff = (vel - observation_map[AgentsPreferredVelocity.name()][0]) ** 2
        return -diff
