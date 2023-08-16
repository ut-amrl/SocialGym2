# !/usr/bin/env python3

import os
import sys
from random import seed
from random import randint
import math
import os
import jinja2
import random
import rospy
import roslib
import json

NODE_NAME = 'ros_social_gym'
roslib.load_manifest(NODE_NAME)
from graph_navigation.srv import graphNavSrv
from amrl_msgs.msg import Pose2Df
from shutil import copyfile

from pathlib import Path
from typing import Union, Tuple, List

from src.environment.scenarios import Scenario

robot_positions = [
    (34.3, 7.04, 1.5708),
    (36.04, 9.06, 3.1459),
    (36.124, 15.972, 3.14),
    (29.15, 15.69, -1.5708),
    (23.49, 6.68, 1.5708),
    (2.03, 6.68, 1.5708),
    (-6.38, 6.38, 1.5708),
    (-10.06, 6.36, 1.5708),
    (-13.68, 6.36, 1.5708),
    (-19, 6.36, 1.5708),
    (-30.05, 4.27, 0),
    (-30.05, 13.95, 0),
    (-38.1, 15.8, 0),
    (-36.1, 18.18, -1.5708),
    (-36.1, 20.65, 0),
    (-24.39, 16.1, 3.14),
    (-20.78, 10.35, -1.5708),
    (-14.38, 10.35, -1.5708),
    (-1.45, 10.35, -1.5708),
    (0.74, 10.35, -1.5708),
    (9.9, 10.35, -1.5708),
    (15.09, 10.35, -1.5708),
    (19.75, 16.35, 3.14),
    (14.03, 24.11, 0),
    (-1.6, 16.24, -1.5708),
    (-13.71, 16.24, 0)
]

nav_map = [
    (38.394, 19.271),
    (28.734, 19.777),
    (29.069, 8.621),
    (9.770, 8.735),
    (15.182, 6.621),
    (14.954, 8.704),
    (14.836, 12.641),
    (39.381, 21.205),
    (36.060, 9.116),
    (34.291, 8.973),
    (34.234, 7.491),
    (32.790, 8.820),
    (32.742, 7.309),
    (23.622, 8.619),
    (23.660, 7.109),
    (21.470, 8.686),
    (21.499, 7.233),
    (19.616, 8.877),
    (19.683, 10.655),
    (13.841, 7.013),
    (13.822, 8.705),
    (9.893, 10.780),
    (7.503, 6.783),
    (7.531, 8.552),
    (6.097, 8.581),
    (6.145, 7.003),
    (2.340, 8.676),
    (2.311, 7.128),
    (0.715, 10.187),
    (0.734, 8.648),
    (-1.496, 10.413),
    (-0.324, 10.437),
    (-1.393, 11.857),
    (-0.116, 11.841),
    (-1.454, 12.744),
    (-3.043, 12.608),
    (-3.009, 14.418),
    (-3.001, 11.036),
    (-1.710, 15.093),
    (-1.700, 16.353),
    (-1.607, 18.646),
    (-13.858, 19.090),
    (-13.889, 15.868),
    (-13.827, 12.924),
    (-13.827, 8.461),
    (-13.703, 6.581),
    (-10.046, 8.182),
    (-10.087, 6.220),
    (-6.431, 8.244),
    (-6.410, 6.540),
    (-4.799, 8.420),
    (-4.768, 7.118),
    (-18.940, 8.172),
    (-18.909, 6.292),
    (-20.737, 8.554),
    (-20.685, 10.538),
    (-20.613, 16.374),
    (-24.559, 16.208),
    (-26.563, 16.095),
    (-26.708, 13.967),
    (-30.054, 14.060),
    (-28.210, 18.521),
    (-34.461, 20.710),
    (-36.486, 20.754),
    (-35.807, 15.948),
    (-35.862, 17.656),
    (-38.216, 15.817),
    (-26.991, 4.441),
    (-29.759, 4.355),
    (-36.514, 4.542),
    (-36.815, 14.137),
    (-13.681, 4.700),
    (-9.952, 4.614),
    (-1.347, 8.672),
    (13.836, 21.237),
    (13.859, 24.275),
    (13.955, 30.517)
]


class GraphNavScenario(Scenario):
    def __init__(
            self,
            env_name: str,
            allowed_agent_start_positions: List[int] = None,
            allowed_agent_goal_positions: List[int] = None,
            allowed_human_start_positions: List[int] = None,
            allowed_human_goal_positions: List[int] = None
    ):
        super().__init__(env_name=env_name)

        self.allowed_agent_start_positions = allowed_agent_start_positions
        self.allowed_agent_goal_positions = allowed_agent_goal_positions
        self.allowed_human_start_positions = allowed_human_start_positions
        self.allowed_human_goal_positions = allowed_human_goal_positions

    def generate_scenario(
            self,
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = 1
    ):
        global robot_positions
        global nav_map
        self.nav_map = nav_map
        self.robot_positions = robot_positions

        if self.config_nav_path is not None:
            self.nav_map, self.nav_lines = self.load_nav_nodes(self.config_nav_path)
            self.robot_positions = nav_map

        if isinstance(num_humans, tuple):
            num_humans = randint(num_humans[0], num_humans[1])

        if isinstance(num_agents, tuple):
            num_agents = randint(num_agents[0], num_agents[1])

        if self.allowed_human_goal_positions:
            robot_starts = random.sample(self.allowed_agent_start_positions, num_agents)
        else:
            robot_starts = random.sample(range(0, len(self.robot_positions)), num_agents)

        if self.allowed_agent_goal_positions:
            robot_ends = [random.sample(list(set(self.allowed_human_goal_positions) - {x}), 1) for x in enumerate(robot_starts)]
        else:
            # robot_ends = [random.sample(list(set(list(range(0, len(self.robot_positions)))) - {x}), 1) for x in enumerate(robot_starts)]
            robot_ends = [random.sample(list(set(list(range(0, len(self.robot_positions)))) - {x}), 1)[0] for x in enumerate(robot_starts)]

        human_positions = []

        if self.allowed_human_start_positions:
            allowed_human_starts = self.allowed_human_start_positions
        else:
            allowed_human_starts = list(range(0, len(self.robot_positions)))

        # allowed_human_starts = list(set(allowed_human_starts) - {robot_start})

        for i in range(0, num_humans):
            human_start = random.sample(allowed_human_starts, 1)[0]

            if self.allowed_human_goal_positions:
                human_end = random.sample(list(set(self.allowed_human_goal_positions) - {human_start}), 1)
            else:
                human_end = random.sample(list(set(list(range(0, len(self.robot_positions)))) - {human_start}), 1)
            human_end = human_end[0]

            planner = rospy.ServiceProxy('graphNavSrv', graphNavSrv)
            rospy.wait_for_service('graphNavSrv')
            h_start = self.robot_positions[human_start]
            h_end = self.robot_positions[human_end]
            start = Pose2Df(h_start[0], h_start[1], 0)
            end = Pose2Df(h_end[0], h_end[1], 0)
            resp = planner(start, end)
            plan = [x for x in resp.plan if x < len(nav_map)]
            r_plan = plan[::-1]
            human_list = [self.robot_positions[human_start][0],
                          self.robot_positions[human_start][1],
                          human_end]
            human_list.extend(r_plan)
            human_list.extend(plan)
            human_positions.append(human_list)
        # Build the Config Dictionary
        human_dev = 1.0
        config = {
            'robot_start': [self.robot_positions[x] for x in robot_starts],
            'robot_end': [self.robot_positions[x] for x in robot_ends],
            'human_count': num_humans,
            'position_count': len(self.robot_positions),
            'nav_count': len(self.nav_map),
            'dev': human_dev,
            'positions': self.robot_positions,
            'human_positions': human_positions,
            'nav_map': self.nav_map,
            'robot_count': num_agents
        }
        self.make_scenario(config)
