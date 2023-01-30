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
            partially_observable: bool = True,
            config_runner: bool = False,
            all_config: bool = False
    ):
        super().__init__(env_name=env_name, partially_observable=partially_observable, config_runner=config_runner, all_config=all_config)

        self.iter = 0

    def generate_scenario(
            self,
            num_humans: Union[int, Tuple[int, int]] = 0,
            num_agents: Union[int, Tuple[int, int]] = 1
    ):

        assert num_humans == 0 or max(num_humans) == 0, 'No humans are allowed in the cycle scenario'
        assert num_agents == 1 or max(num_agents) == 1, 'Only one robot is allowed in the cycle scenario'

        if self.config_nav_path is not None:
            self.nav_map, self.nav_lines = self.load_nav_nodes(self.config_nav_path)
            self.robot_positions = self.nav_map
        else:
            raise Exception("Cycle scenarios need a custom map")

        robot_starts = [self.iter]
        robot_ends = [self.iter]

        # Build the Config Dictionary
        human_dev = 1.0
        config = {
            'robot_start': [self.robot_positions[x] for x in robot_starts],
            'robot_end': [self.robot_positions[x] for x in robot_ends],
            'robot_count': 1,
            'human_count': 0,
            'position_count': len(self.robot_positions),
            'nav_count': len(self.nav_map),
            'dev': human_dev,
            'positions': self.robot_positions,
            'human_positions': [],
            'nav_map': self.nav_map
        }
        self.make_scenario(config)

        print(f"Cycle Scenario: Iter index {self.iter}. Robot at node {robot_starts[0]}.")

        self.iter = self.iter + 1 if self.iter + 1 < len(self.robot_positions) else 0

        if self.iter == 0:
            print("LAST NODE - resetting iter index")
