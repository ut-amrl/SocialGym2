from src.environment.utils import poses_to_np_array
from ut_multirobot_sim.srv import utmrsStepper
from ut_multirobot_sim.srv import utmrsReset

import rospy
import roslib
from typing import List, Tuple, ClassVar
import numpy as np


class UTMRSResponse:
    collision: bool
    done: bool
    door_pose: np.array
    door_state: int
    follow_target: int
    goal_pose: np.array
    human_poses: List[np.array]
    human_vels: List[np.array]
    local_target: np.array
    other_robot_poses: List[np.array]
    other_robot_vels: List[np.array]
    robot_poses: np.array
    robot_state: int
    robot_vels: np.array
    success: bool

    @classmethod
    def process(cls, env_response, *args, **kwargs) -> List['UTMRSResponse']:
        observations = []

        for robot_idx, robot_obs in enumerate(env_response.robot_responses):
            inst = cls()
            inst.set_vars(robot_obs, robot_idx, *args, **kwargs)
            observations.append(inst)
        return observations

    def set_vars(self, robot_obs, robot_idx: int, *args, **kwargs):
        self.collision = robot_obs.collision
        self.done = robot_obs.done
        self.door_pose = poses_to_np_array(robot_obs.door_pose)
        self.door_state = robot_obs.door_state
        self.follow_target = robot_obs.follow_target
        self.goal_pose = poses_to_np_array(robot_obs.goal_pose)
        self.human_poses = [poses_to_np_array(x) for x in robot_obs.human_poses] or []
        self.human_vels = [poses_to_np_array(x) for x in robot_obs.human_vels] or []
        self.local_target = poses_to_np_array(robot_obs.local_target)
        self.other_robot_poses = [poses_to_np_array(x) for x in robot_obs.other_robot_poses] or []
        self.other_robot_vels = [poses_to_np_array(x) for x in robot_obs.other_robot_vels] or []
        self.robot_poses = poses_to_np_array(robot_obs.robot_poses)
        self.robot_state = robot_obs.robot_state
        self.robot_vels = poses_to_np_array(robot_obs.robot_vels)
        self.success = robot_obs.success


class UTMRS:
    
    def __init__(self):
        rospy.wait_for_service('utmrsStepper')
        rospy.wait_for_service('utmrsReset')

        self.sim_step = rospy.ServiceProxy('utmrsStepper', utmrsStepper)
        self.sim_reset = rospy.ServiceProxy('utmrsReset', utmrsReset)

    def step(self, *args) -> List[UTMRSResponse]:
        return self.sim_step(*args)

    def reset(self, *args) -> None:
        self.sim_reset(*args)
