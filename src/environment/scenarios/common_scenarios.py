from itertools import product

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
    start_points = [6, 18, 17, 0, 10, 3, 8, 15]
    before_hallway = [7]
    after_hallway = [11]
    end_points = [5, 14, 4, 16, 9, 2, 13]

    all_paths = [list(x) for x in product(start_points, before_hallway, after_hallway, end_points)]

    # TODO - exp1 isn't right??
    scenario = ManualScenario(
        f'exp1/train/{level}',
        agent_paths=all_paths,
        human_paths=all_paths,
        partially_observable=partially_observable,
        config_runner=config_runner,
        all_config=all_config
    )

    return scenario

