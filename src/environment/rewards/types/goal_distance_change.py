import numpy as np
from typing import Dict

from src.environment.observations import AgentsGoalDistance
from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import poses_to_np_array


class GoalDistanceChange(Reward):
    """
    Simple reward that returns the change in distance to the goal per timestep as a reward.

    I.E.

    timestep 1 -> 5 units to the goal, will return 0
    timestep 2 -> 3 units to the goal, will return 5 - 3 -> 2
    timestep 3 -> 4 units to the goal, will return 3 - 4 -> -1
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

        self.last_goal_distance = -1

    @classmethod
    def name(cls):
        return "goal_distance_change"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array], data_map) -> float:
        assert AgentsGoalDistance.name() in observation_map, \
            'Goal Distance requires the agents goal distance to be in the observation'
        assert len(observation_map[AgentsGoalDistance.name()]) > 1, \
            'Goal Distance requires that the AgentGoalDistance has at least a history length of 2.'

        dists_to_goal = observation_map[AgentsGoalDistance.name()]
        score = dists_to_goal[-2] - dists_to_goal[-1]

        return score
