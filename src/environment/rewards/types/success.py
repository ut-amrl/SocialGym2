import numpy as np
from typing import Dict

from src.environment.observations import SuccessObservation, AgentsGoalDistance
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class Success(Reward):
    """
    Reward for when the agent has successfully completed the goal in the env.
    """

    def __init__(self, weight: float = 100.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "success"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        assert SuccessObservation.name() in observation_map, \
           'The Success Reward expects the SuccessObservation to be given'

        return 1. if observation_map[SuccessObservation.name()] > 0 else 0.
