import math

import numpy as np
from typing import Dict, Tuple

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import poses_to_np_array
from src.environment.observations import AgentsGoalDistance, AgentsOthersDistance, AgentsHeadingDirection, \
    OthersHeadingDirection, AgentsVelocity, OthersVelocities


class SocialNormCross(Reward):
    """
    Social Norm Crossing Penalty as defined in https://arxiv.org/pdf/1703.08862.pdf on page 4 eq 12
    """

    goal_dist_threshold: float
    dist_to_other_threshold: float
    relative_agent_to_other_angle_threshold: float
    heading_angle_thresholds: Tuple[float, float]

    def __init__(
            self,
            weight: float = -1.0,
            goal_dist_threshold: float = 3.0,
            dist_to_other_threshold: float = 2.0,
            relative_agent_to_other_angle_threshold: float = 0.,
            heading_angle_thresholds: Tuple[float, float] = (-3 * math.pi / 4, -math.pi / 4),
            *args,
            **kwargs
    ):
        super().__init__(weight)

        self.goal_dist_threshold = goal_dist_threshold
        self.dist_to_other_threshold = dist_to_other_threshold
        self.relative_agent_to_other_angle_threshold = relative_agent_to_other_angle_threshold
        self.heading_angle_thresholds = heading_angle_thresholds

    @classmethod
    def name(cls):
        return "social_norm_cross"

    def __score__(self, env: RosSocialEnv, env_response, observation_map: Dict[str, np.array], data_map) -> float:
        assert AgentsGoalDistance.name() in observation_map, \
            'Social Norm passing requires the agents goal distance to be in the observation'
        assert AgentsOthersDistance.name() in observation_map, \
            'Social Norm passing requires the agents distance to other agents/humans in the observation'
        assert AgentsHeadingDirection.name() in observation_map, \
            'Social Norm Passing requires the agents heading direction to be in the observation'
        assert OthersHeadingDirection.name() in observation_map, \
            'Social Norm Passing requires the other agents/humans heading directions to be in the observation'
        assert OthersVelocities.name() in observation_map, \
            'Social Norm Cross requires the other agents/humans velocities to be in the observation'
        assert AgentsVelocity.name() in observation_map, \
            'Social Norm Cross requires that the agents velocities are in the observation'

        if self.goal_dist_threshold < observation_map[AgentsGoalDistance.name()][0]:
            return 0.0

        agent_dists = observation_map[AgentsOthersDistance.name()].reshape(-1, 1)
        agents_hd = observation_map[AgentsHeadingDirection.name()]
        other_agents_hd = observation_map[OthersHeadingDirection.name()]

        agent_vels = observation_map[AgentsVelocity.name()][0:2]
        other_vels = observation_map[OthersVelocities.name()].reshape((-1, 3))[:, 0:2]

        v_diffs = other_vels - agent_vels
        relative_angles = np.arctan(v_diffs[:, 0] / v_diffs[:, 1])

        relative_angle_check = relative_angles > self.relative_agent_to_other_angle_threshold

        dist_check = agent_dists < self.dist_to_other_threshold

        heading_diff = other_agents_hd - agents_hd
        heading_check = (self.heading_angle_thresholds[0] < heading_diff) & \
                        (heading_diff < self.heading_angle_thresholds[1])

        num_of_violations = (dist_check & heading_check & relative_angle_check).sum()

        return num_of_violations
