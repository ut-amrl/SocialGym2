import numpy as np
from typing import Dict

from src.environment.observations import AgentsGoalDistance
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv


class GoalDistance(Reward):
    """
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

        self.last_goal_distance = -1

    @classmethod
    def name(cls):
        return "goal_distance"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array], data_map) -> float:
        assert AgentsGoalDistance.name() in observation_map, \
            'Goal Distance requires the agents goal distance to be in the observation'

        dists_to_goal = observation_map[AgentsGoalDistance.name()][-1]
        score = 1/dists_to_goal

        return score
