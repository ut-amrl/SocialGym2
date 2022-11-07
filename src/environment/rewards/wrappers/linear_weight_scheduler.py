from abc import ABC
import numpy as np
from typing import Dict

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class LinearWeightScheduler(Reward, ABC):
    """
    linearly interpolates the weight value between min_weight and max_weight starting at min_weight then incrementally
    increasing the weight until max weight is achieved.

    Only works for increasing weights not decreasing - TODO - this is probably easy.
    """

    def __init__(self, reward: Reward, min_weight: float = 1.0, max_weight: float = 1.0, duration: int = 1000, *args, **kwargs):
        super().__init__(min_weight)
        self.reward = reward
        self.reward.weight = 1.0

        self.min_weight = min_weight
        self.max_weight = max_weight
        self.step = duration

    @classmethod
    def name(cls):
        return "linear_weight_scheduler"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        score = self.reward.__score__(env, observation_map)
        return score

    def __reset__(self):
        self.reward.__reset__()
        self.weight = self.min_weight + (self.max_weight - self.min_weight) / self.step

        if self.step > 1:
            self.step -= 1
        else:
            self.step = 1
