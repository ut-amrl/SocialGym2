import numpy as np

from src.environment.rewards import Reward
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import poses_to_np_array


class GoalDistance(Reward):
    """
    Simple reward that returns the change in distance to the goal per timestep as a reward.

    I.E.

    timestep 1 -> 5 units to the goal, will return 5
    timestep 2 -> 3 units to the goal, will return 5 - 3 -> 2
    timestep 3 -> 4 units to the goal, will return 3 - 4 -> -1
    """

    def __init__(self, weight: float = 1.0, *args, **kwargs):
        super().__init__(weight)

        self.last_goal_distance = -1

    def __score__(self, env: RosSocialEnv, env_response, data_map) -> float:
        robot_poses = poses_to_np_array(env_response.robot_poses)
        goal_pose = poses_to_np_array(env_response.goal_pose)

        goal_distance = np.linalg.norm(robot_poses[0] - goal_pose[0])

        if self.last_goal_distance == -1:
            # First time-step, no progress has been made and returning the current distance would give a large bump
            # for nothing.
            # TODO - is this right?
            distance_score = 0
        else:
            distance_score = self.last_goal_distance - goal_distance

        self.last_goal_distance = goal_distance

        # TODO - should we be updating these here?
        env.last_goal_distance = goal_distance
        data_map['DistScore'] = distance_score

        return distance_score
