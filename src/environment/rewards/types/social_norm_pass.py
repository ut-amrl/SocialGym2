import math

import numpy as np
from typing import Dict, Tuple

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import poses_to_np_array
from src.environment.observations import AgentsGoalDistance, OthersPoses, AgentsHeadingDirection, OthersHeadingDirection


class SocialNormPass(Reward):
    """
    Social Norm Pass Penalty as defined in https://arxiv.org/pdf/1703.08862.pdf on page 4 eq 10
    """

    goal_dist_threshold: float
    px_threshold: Tuple[float, float]
    py_threshold: Tuple[float, float]
    heading_angle_threshold: float

    def __init__(
            self,
            weight: float = -1.0,
            goal_dist_threshold: float = 3.0,
            px_threshold: Tuple[float, float] = (1, 4),
            py_threshold: Tuple[float, float] = (-2, 0),
            heading_angle_threshold: float = 3 * math.pi / 4,
            *args,
            **kwargs
    ):
        super().__init__(weight)

        self.goal_dist_threshold = goal_dist_threshold
        self.px_threshold = px_threshold
        self.py_threshold = py_threshold
        self.heading_angle_threshold = heading_angle_threshold

    @classmethod
    def name(cls):
        return "social_norm_pass"

    def __score__(self, env: RosSocialEnv, env_response, observation_map: Dict[str, np.array], data_map) -> float:
        assert AgentsGoalDistance.name() in observation_map, \
            'Social Norm passing requires the agents goal distance to be in the observation'
        assert OthersPoses.name() in observation_map, \
            'Social Norm passing requires the other agents/humans poses to be in the observation'
        assert AgentsHeadingDirection.name() in observation_map, \
            'Social Norm Passing requires the agents heading direction to be in the observation'
        assert OthersHeadingDirection.name() in observation_map, \
            'Social Norm Passing requires the other agents/humans heading directions to be in the observation'

        if self.goal_dist_threshold < observation_map[AgentsGoalDistance.name()][0]:
            return 0.0

        poses = observation_map[OthersPoses.name()].reshape((-1, 3))[:, 0:2]

        agents_hd = observation_map[AgentsHeadingDirection.name()]
        other_agents_hd = observation_map[OthersHeadingDirection.name()]

        x_check = (self.px_threshold[0] < poses[:, 0]) & (poses[:, 0] < self.px_threshold[1])
        y_check = (self.py_threshold[0] < poses[:, 1]) & (poses[:, 1] < self.py_threshold[1])

        heading_diff_magnitudes = np.absolute(other_agents_hd - agents_hd)
        heading_check = self.heading_angle_threshold < heading_diff_magnitudes

        num_of_violations = (x_check & y_check & heading_check).sum()

        return num_of_violations
