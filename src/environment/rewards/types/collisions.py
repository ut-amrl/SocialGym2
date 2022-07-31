import numpy as np
from typing import Dict

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class Collisions(Reward):
    """
    Penalty for when the agent collides with something in the environment.
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "collisions"

    def __score__(self, env: RosSocialEnv, env_response, observation_map: Dict[str, np.array], data_map) -> float:
        # We return -1. or 0. here and let the weight determine how much emphasis should be put on collisions.
        return -1. if env_response.collision else 0.
