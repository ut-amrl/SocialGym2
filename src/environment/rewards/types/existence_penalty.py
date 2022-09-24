import numpy as np
from typing import Dict

from src.environment.observations import SuccessObservation
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class ExistencePenalty(Reward):
    """
    Penalty for every timestep that has not led to success.
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "existence_penalty"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        return 0. if observation_map.get(SuccessObservation.name(), 0) > 0 else -1.
