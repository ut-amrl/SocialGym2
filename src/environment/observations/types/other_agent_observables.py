import numpy as np

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.observations import Observation
from src.environment.services import UTMRSResponse


class OtherAgentObservables(Observation):
    """
    """

    num_others: int

    def __init__(
            self,
            pos_x: bool = True,
            pos_y: bool = True,
            pos_theta: bool = False,
            vel_x: bool = True,
            vel_y: bool = True,
            vel_theta: bool = False

    ):
        super().__init__()

        self.num_others = 0

        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_theta = pos_theta
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_theta = vel_theta

        self.agent_obs_length = 0
        if self.pos_x:
            self.agent_obs_length += 1
        if self.pos_y:
            self.agent_obs_length += 1
        if self.pos_theta:
            self.agent_obs_length += 1
        if self.vel_x:
            self.agent_obs_length += 1
        if self.vel_y:
            self.agent_obs_length += 1
        if self.vel_theta:
            self.agent_obs_length += 1

    @classmethod
    def name(cls):
        return "other_agent_observables"

    def __len__(self):
        return self.num_others * self.agent_obs_length

    def __observations__(self, env: RosSocialEnv, env_response: UTMRSResponse) -> np.array:
        other_obs = np.zeros([len(self)])

        poses = [*env_response.other_robot_poses, *env_response.human_poses]
        vels = [*env_response.other_robot_vels, *env_response.human_vels]

        for i in range(len(poses)):
            start_idx = i * self.agent_obs_length

            cur_pose = poses[i]
            cur_vel = vels[i]

            agent_obs = []
            if self.pos_x:
                agent_obs.append(cur_pose[0])
            if self.pos_y:
                agent_obs.append(cur_pose[1])
            if self.pos_theta:
                agent_obs.append(cur_pose[2])
            if self.vel_x:
                agent_obs.append(cur_vel[0])
            if self.vel_y:
                agent_obs.append(cur_vel[1])
            if self.vel_theta:
                agent_obs.append(cur_vel[2])

            other_obs[start_idx:(i+1)*self.agent_obs_length] = agent_obs

        return other_obs

    def __setup__(self, env: RosSocialEnv):
        self.num_others = 0
        self.num_others += max(0, max(env.ros_num_humans))
        self.num_others += max(0, max(env.ros_num_agents) - 1)

