# !/usr/bin/env python3

from copy import deepcopy
import os
import sys
from random import seed
from random import randint
import math
import os
import jinja2
import random
import json

from shutil import copyfile

from pathlib import Path
from typing import Union, Tuple, List


from src.environment.scenarios import Scenario


class ManualScenario(Scenario):
    def __init__(
            self,
            env_name: str,
            agent_paths: List[List[int]],
            human_paths: List[List[int]],
            partially_observable: bool = True,
            config_runner: bool = False,
    ):
        super().__init__(env_name=env_name, partially_observable=partially_observable, config_runner=config_runner)

        self.agent_paths = agent_paths
        self.human_paths = human_paths

    def generate_scenario(
            self,
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = 1
    ):
        if self.config_nav_path is not None:
            self.nav_map = self.load_nav_nodes(self.config_nav_path)
            self.robot_positions = self.nav_map
        else:
            raise Exception("Manual scenarios need a custom map")

        if isinstance(num_humans, tuple):
            num_humans = randint(num_humans[0], num_humans[1])

        if isinstance(num_agents, tuple):
            num_agents = randint(num_agents[0], num_agents[1])

        allowed_robot_paths = deepcopy(self.agent_paths)
        robot_paths = []
        for i in range(num_agents):
            picked_path = random.sample(allowed_robot_paths, 1)[0]
            robot_paths.append(picked_path)
            allowed_robot_paths = [x for x in allowed_robot_paths if x[0] != picked_path[0]]
            allowed_robot_paths = [x for x in allowed_robot_paths if x[-1] != picked_path[-1]]

        robot_starts = [x[0] for x in robot_paths]
        robot_ends = [x[-1] for x in robot_paths]

        human_positions = []

        # TODO - ask jarrett, but maybe having the same starting point is ok.
        allowed_human_paths = self.human_paths  # [x for x in self.human_paths if x[0] == robot_start]

        if len(allowed_human_paths) > 0:
            for i in range(0, num_humans):
                human_path = random.sample(allowed_human_paths, 1)[0]
                human_start = human_path[0]
                human_end = human_path[-1]

                human_list = [self.robot_positions[human_start][0],
                              self.robot_positions[human_start][1],
                              human_end]

                human_list.extend(list(reversed(human_path)))
                human_list.extend(human_path)
                human_positions.append(human_list)
        # Build the Config Dictionary
        human_dev = 1.0
        config = {
            'robot_start': [self.robot_positions[x] for x in robot_starts],
            'robot_end': [self.robot_positions[x] for x in robot_ends],
            'robot_count': len(robot_paths),
            'human_count': num_humans,
            'position_count': len(self.robot_positions),
            'nav_count': len(self.nav_map),
            'dev': human_dev,
            'positions': self.robot_positions,
            'human_positions': human_positions,
            'nav_map': self.nav_map
        }
        self.make_scenario(config)
