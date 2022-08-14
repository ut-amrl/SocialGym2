from src.environment.scenarios import Scenario, GraphNavScenario, ManualScenario


def closed_door_1__opposing_sides() -> Scenario:
    allowed_agent_start_positions = [5]
    allowed_agent_goal_positions = [0, 5]

    allowed_human_start_positions = [0, 5]
    allowed_human_goal_positions = [1, 3]

    scenario = GraphNavScenario(
        'closed/door/t1',
        allowed_agent_start_positions=allowed_agent_start_positions,
        allowed_agent_goal_positions=allowed_agent_goal_positions,
        allowed_human_start_positions=allowed_human_start_positions,
        allowed_human_goal_positions=allowed_human_goal_positions
    )

    return scenario


def closed_door_1__same_goals() -> Scenario:
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
        'closed/door/t1',
        agent_paths=allowed_robot_paths,
        human_paths=allowed_human_paths
    )

    return scenario
