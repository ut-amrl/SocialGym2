import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict, Callable
from pettingzoo.utils.wrappers import BaseParallelWraper
import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from src.environment.utils import get_tboard_writer

bridge = CvBridge()


class TensorboardWriter(BaseParallelWraper):
    """
    """

    def __init__(
            self,
            env: gym.Env,
            tbx_log: str,
            fps: int = 30,
            max_video_length: int = 10_000,
            video_name: str = 'Episode',
            record_video: bool = True,
            record_rewards: bool = True,
            record_env_info: bool = True
    ):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)
        self.tbx_log = tbx_log
        self.tbx_writer = None

        self.last_image = None
        self.last_image_ts = 0
        self.video = []
        self.max_vid_length = max_video_length
        self.fps = fps
        self.video_name = video_name

        self.step_count = 0
        self.total_step_count = 0
        self.episode_count = 0
        self.number_of_collisions = 0
        self.successes = 0
        self.velocity_changes = 0

    def image_callback(self, msg):
        if self.last_image_ts != self.step_count and len(self.video) < self.max_vid_length:
            cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
            self.video.append(cv2_img.transpose(2, 0, 1))
            self.last_image_ts = self.step_count

    def step(self, actions):
        self.step_count += 1
        self.total_step_count += 1

        last_obs = self.unwrapped.last_obs_maps

        self.tbx_writer.add_scalars('rewards/scalars', {f'{i}.{k}': v for i, agents_rewards in enumerate(self.unwrapped.last_reward_maps) for k, v in agents_rewards.items()}, self.total_step_count)

        self.number_of_collisions += sum([1 if x['collisions'] else 0 for x in last_obs])
        self.successes = 1 if all([x.get('success_observation', 0) for x in last_obs]) else 0
        self.velocity_changes += sum([abs(x['agents_velocity'][0:2] - x['agents_velocity'][3:5]).sum() for x in last_obs])

        res = self.env.step(actions)
        self.agents = self.env.agents
        return res

    def reset(self, seed=None, return_info=False, options=None):
        if not self.tbx_writer:
            self.tbx_writer = get_tboard_writer(self.tbx_log)
            image_topic = "/rviz1/camera1/image"
            rospy.Subscriber(image_topic, Image, self.image_callback)

        if len(self.video) > 0:
            self.tbx_writer.add_video(self.video_name, np.stack([np.stack(self.video)]), self.episode_count, fps=self.fps)

        self.tbx_writer.add_scalars('env_info', {
            'step_count': self.step_count,
            'episode_count': self.episode_count,
            'number_of_collisions': self.number_of_collisions,
            'successes': self.successes,
            'velocity_changes': int(self.velocity_changes)
        }, self.episode_count)

        self.number_of_collisions = 0
        self.successes = 0
        self.velocity_changes = 0
        self.step_count = 0
        self.episode_count += 1
        self.video = []

        if not return_info:
            res = self.env.reset(seed=seed, options=options)
            self.agents = self.env.agents
            return res
        else:
            res, info = self.env.reset(
                seed=seed, return_info=return_info, options=options
            )
            self.agents = self.env.agents
            return res, info

    def seed(self, seed=None):
        pass
