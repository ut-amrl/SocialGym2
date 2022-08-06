import numpy as np
from typing import Dict

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import CollisionObservation


class Collisions(Reward):
    """
    Penalty for when the agent collides with something in the environment.
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

    @classmethod
    def name(cls):
        return "collisions"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array], data_map) -> float:
        assert CollisionObservation.name() in observation_map, \
            'Collisions reward expects teh CollisionObservation in the observation.'
        return -1.0 if observation_map[CollisionObservation.name()] > 0 else 0.
