# !/usr/bin/env python3

from random import randint
from typing import Union, Tuple
from src.environment.scenarios import Scenario


class CycleScenario(Scenario):
    """
    A simple DEBUG scenario that will cycle through all the nodes in a graph nav file choosing a new node per episode
    until it cycles.

    Useful for knowing which node on the graph nav is what ID for manual planning.
    """

    def __init__(
            self,
            env_name: str,
    ):
        super().__init__(env_name=env_name)

        self.iter = 0

    def generate_scenario(
            self,
            num_humans: Union[int, Tuple[int, int]] = 0,
            num_agents: Union[int, Tuple[int, int]] = 1
    ):

        assert num_humans == 0 or max(num_humans) == 0, 'No humans are allowed in the cycle scenario'
        assert num_agents == 1 or max(num_agents) == 1, 'Only one robot is allowed in the cycle scenario'

        if self.config_nav_path is not None:
            nav_map = self.load_nav_nodes(self.config_nav_path)
            robot_positions = nav_map
        else:
            raise Exception("Cycle scenarios need a custom map")

        robot_starts = [self.iter]
        robot_ends = [self.iter]

        # Build the Config Dictionary
        human_dev = 1.0
        config = {
            'robot_start': [robot_positions[x] for x in robot_starts],
            'robot_end': [robot_positions[x] for x in robot_ends],
            'robot_count': 1,
            'human_count': 0,
            'position_count': len(robot_positions),
            'nav_count': len(nav_map),
            'dev': human_dev,
            'positions': robot_positions,
            'human_positions': [],
            'nav_map': nav_map
        }
        self.make_scenario(config)

        print(f"Cycle Scenario: Iter index {self.iter}. Robot at node {robot_starts[0]}.")

        self.iter = self.iter + 1 if self.iter + 1 < len(robot_positions) else 0

        if self.iter == 0:
            print("LAST NODE - resetting iter index")
