import math

import numpy as np
from typing import Dict, Tuple

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import AgentsGoalDistance, OthersPoses, AgentsHeadingDirection,\
    OthersHeadingDirection, AgentsVelocity, OthersVelocities, AgentsPose


class SocialNormOvertake(Reward):
    """
    Social Norm Overtake Penalty as defined in https://arxiv.org/pdf/1703.08862.pdf on page 4 eq 11
    """

    goal_dist_threshold: float
    px_threshold: Tuple[float, float]
    py_threshold: Tuple[float, float]
    heading_angle_threshold: float

    def __init__(
            self,
            weight: float = -1.0,
            goal_dist_threshold: float = 3.0,
            px_threshold: Tuple[float, float] = (0, 3),
            py_threshold: Tuple[float, float] = (0, 1),
            heading_angle_threshold: float = math.pi / 4,
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
        return "social_norm_overtake"

    def __score__(self, env: RosSocialEnv, observation_map: Dict[str, np.array]) -> float:
        assert AgentsGoalDistance.name() in observation_map, \
            'Social Norm Overtake requires the agents goal distance to be in the observation'
        assert OthersPoses.name() in observation_map, \
            'Social Norm Overtake requires the other agents/humans poses to be in the observation'
        assert AgentsPose.name() in observation_map, \
            'Social Norm passing requires the agents pose to be in the observation'
        assert AgentsHeadingDirection.name() in observation_map, \
            'Social Norm Overtake requires the agents heading direction to be in the observation'
        assert OthersHeadingDirection.name() in observation_map, \
            'Social Norm Overtake requires the other agents/humans heading directions to be in the observation'
        assert OthersVelocities.name() in observation_map, \
            'Social Norm Overtake requires the other agents/humans velocities to be in the observation'
        assert AgentsVelocity.name() in observation_map, \
            'Social Norm Overtake requires that the agents velocities are in the observation'

        if self.goal_dist_threshold < observation_map[AgentsGoalDistance.name()][0]:
            return 0.0

        poses = observation_map[OthersPoses.name()].reshape((-1, 3))[:, 0:2]
        agents_pose = observation_map[AgentsPose.name()][0:2]

        # TODO - paper doesn't say to use the difference between the agent and the other human/agent but it doesn't
        #   make since otherwise
        # pose_diffs = poses - agents_pose
        # TODO - are the poses relative to the agent already?
        pose_diffs = poses

        agents_hd = observation_map[AgentsHeadingDirection.name()]
        other_agents_hd = observation_map[OthersHeadingDirection.name()]

        agents_vel = np.linalg.norm(observation_map[AgentsVelocity.name()][0:2])
        other_vels = np.linalg.norm(observation_map[OthersVelocities.name()].reshape(-1, 3)[:, 0:2], axis=-1)

        x_check = (self.px_threshold[0] < pose_diffs[:, 0]) & (pose_diffs[:, 0] < self.px_threshold[1])
        y_check = (self.py_threshold[0] < pose_diffs[:, 1]) & (pose_diffs[:, 1] < self.py_threshold[1])

        heading_diff_magnitudes = np.absolute(other_agents_hd - agents_hd)
        heading_check = self.heading_angle_threshold < heading_diff_magnitudes

        vel_checks = agents_vel > other_vels

        num_of_violations = (x_check & y_check & heading_check & vel_checks).sum()

        return -num_of_violations
