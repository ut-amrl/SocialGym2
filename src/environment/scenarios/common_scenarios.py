from itertools import product
from typing import Tuple

from src.environment.scenarios import Scenario, GraphNavScenario, ManualScenario


def closed_door_1__opposing_sides(task_id: str = 't1') -> Scenario:
    allowed_agent_start_positions = [5]
    allowed_agent_goal_positions = [0, 5]

    allowed_human_start_positions = [0, 5]
    allowed_human_goal_positions = [1, 3]

    scenario = GraphNavScenario(
        f'closed/door/{task_id}',
        allowed_agent_start_positions=allowed_agent_start_positions,
        allowed_agent_goal_positions=allowed_agent_goal_positions,
        allowed_human_start_positions=allowed_human_start_positions,
        allowed_human_goal_positions=allowed_human_goal_positions
    )

    return scenario


def closed_door_1__same_goals(task_id: str = 't1') -> Scenario:
    allowed_robot_paths = [
        [1, 4, 2, 5],
        [1, 4, 2, 0],
        [3, 4, 2, 5],
        [3, 4, 2, 0]
    ]

    allowed_human_paths = [
        [1, 4, 2, 5],
        [1, 4, 2, 0],
        [3, 4, 2, 5],
        [3, 4, 2, 0]
    ]

    scenario = ManualScenario(
        f'closed/door/{task_id}',
        agent_paths=allowed_robot_paths,
        human_paths=allowed_human_paths
    )

    return scenario


def elevator_loading() -> Scenario:
    allowed_robot_paths = [
        [7, 0, 9, 13],
        [7, 0, 9, 11],
        [6, 0, 9, 13, 2, 1],
        [4, 0, 9, 5, 12, 11],
        [6, 0, 9, 13, 11, 12],
    ]

    allowed_human_paths = [
        [5, 5],
        [12, 12],
        [1, 1],
        [2, 2],
        [11, 11],
        [13, 13],
    ]

    scenario = ManualScenario(
        'elevator/t1',
        agent_paths=allowed_robot_paths,
        human_paths=allowed_human_paths
    )

    return scenario


def exp1_train_scenario(level: str = 'easy', partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Scenario:
    start_points = [1, 6, 18, 17, 0, 10, 3, 8, 15, 12]
    before_hallway = [7]
    after_hallway = [11]
    end_points = [19, 5, 14, 4, 16, 9, 21, 2, 13, 20]

    all_paths = [list(x) for x in product(start_points, before_hallway, after_hallway, end_points)]

    scenario = ManualScenario(
        f'exp1/train/{level}',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario

def exp2_train_scenario(level: str = 'easy', partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Scenario:
    starts = [0, 43, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
    before = [20]
    after = [21]
    ends = [22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42]
    all_paths = [list(x) for x in product(starts, before, after, ends)]


    # TODO - exp1 isn't right??
    scenario = ManualScenario(
        # f'exp2/train/{level}',
        f'envs/scenario/door',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario


def envs_door(partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Tuple[Scenario, Tuple[int, int, int]]:
        starts = [0, 43, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        before = [20]
        after = [21]
        ends = [22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42]
        all_paths = [list(x) for x in product(starts, before, after, ends)]

        scenario = ManualScenario(
            f'envs/scenario/door',
            agent_paths=all_paths,
            human_paths=all_paths,
            partially_observable=partially_observable,
            config_runner=config_runner,
            all_config=all_config
        )

        return scenario, (20, 21, 1)


def envs_hallway(partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Tuple[Scenario, Tuple[int, int, int]]:
    starts = list(range(0, 21))
    before = [21]
    after = [22]
    ends = list(range(23, 44))
    all_paths = [list(x) for x in product(starts, before, after, ends)]

    scenario = ManualScenario(
        f'envs/scenario/hallway',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario, (21, 22, 1)


def envs_intersection(partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Tuple[Scenario, Tuple[int, int, int]]:
    starts_b = [0, 1, 30, 2, 3, 4, 31]
    before_b = [29]
    after_t = [27]
    ends_t = [12, 13, 14, 15, 16, 17, 18]
    all_paths = [list(x) for x in product(starts_b, before_b, after_t, ends_t)]

    starts_l = [5, 6, 7, 8, 9, 10, 11]
    before_l = [26]
    after_r = [28]
    ends_r = [19, 20, 21, 22, 23, 24, 25]
    all_paths.extend([list(x) for x in product(starts_l, before_l, after_r, ends_r)])

    scenario = ManualScenario(
        f'envs/scenario/intersection',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario, (26, 28, 2)


def envs_round_about(partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Tuple[Scenario, Tuple[int, int, int]]:

    starts_b = [0, 1, 30, 2, 3, 4, 31]
    before_b = [29]
    after_t = [27]
    ends_t = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 5, 6, 7, 8, 9, 10, 11]
    all_paths = [list(x) for x in product(starts_b, before_b, after_t, ends_t)]

    starts_l = [5, 6, 7, 8, 9, 10, 11]
    before_l = [26]
    after_r = [28]
    ends_r = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 0, 1, 30, 2, 3, 4, 31]
    all_paths.extend([list(x) for x in product(starts_l, before_l, after_r, ends_r)])

    scenario = ManualScenario(
        f'envs/scenario/round_about',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario, (26, 28, 2)


def envs_open(partially_observable: bool = False, config_runner: bool = False, all_config: bool = False) -> Tuple[Scenario, Tuple[int, int, int]]:

    all_paths = []
    for i in range(48):
        all_paths.extend([list(x) for x in product([i], [*list(range(0, i)), *list(range(i+1, 48))])])

    scenario = ManualScenario(
        f'envs/scenario/open',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario, (12, 36, 6)