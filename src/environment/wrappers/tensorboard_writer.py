import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict, Callable
from pettingzoo.utils.wrappers import BaseParallelWraper
import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from src.environment.utils.utils import get_tboard_writer

bridge = CvBridge()


class TensorboardWriter(BaseParallelWraper):
    """
    Handles tensorboard recording of the environments

    Can record video of each episode, the rewards at each timestep, and overall env information per episode.
    """

    def __init__(
            self,
            env: gym.Env,
            tbx_log: str,
            fps: int = 30,
            max_video_length: int = 200,
            video_sample_rate: int = 12,
            video_name: str = 'Episode',
            step_sample_rate: int = 100,
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
        self.video_sample_rate = video_sample_rate
        self.max_vid_length = max_video_length
        self.video = []
        self.fps = fps
        self.video_name = video_name

        self.step_count = 0
        self.total_step_count = 0
        self.episode_count = 0
        self.eval_step_count = 0
        self.eval_total_step_count = 0
        self.eval_episode_count = 0
        self.number_of_collisions = 0
        self.velocity_changes = 0

        self.step_sample_rate = step_sample_rate
        self.record_video = record_video
        self.record_rewards = record_rewards
        self.record_env_info = record_env_info

    def image_callback(self, msg):
        if self.last_image_ts != self.step_count and \
                len(self.video) < self.max_vid_length and \
                (
                        (self.step_count % self.video_sample_rate == 0 and not self.unwrapped.in_eval) or
                        (self.unwrapped.in_eval and self.eval_step_count % self.video_sample_rate == 0)
                ):

            cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
            self.video.append(cv2_img.transpose(2, 0, 1))
            self.last_image_ts = self.step_count

    def step(self, actions):
        res = self.env.step(actions)
        self.agents = self.unwrapped.agents

        if self.unwrapped.in_eval:
            self.eval_step_count += 1
            self.eval_total_step_count += 1
        else:
            self.step_count += 1
            self.total_step_count += 1

        if (not self.unwrapped.in_eval and self.total_step_count % self.step_sample_rate == 0) or\
                (self.unwrapped.in_eval and self.eval_total_step_count % self.step_sample_rate == 0):

            step_count = self.eval_total_step_count if self.unwrapped.in_eval else self.total_step_count

            if len(self.unwrapped.last_reward_maps) > 0 and self.record_rewards:
                if self.unwrapped.in_eval:
                    title = 'eval_rewards/scalars'
                else:
                    title = 'reward/scalars'
                self.tbx_writer.add_scalars(title, {k: sum([x[k] for x in self.unwrapped.last_reward_maps]) / max(self.unwrapped.num_agents, 1) for k in self.unwrapped.last_reward_maps[0].keys()}, step_count)
                self.tbx_writer.add_scalars(title, {'actual': sum(res[1].values())}, step_count)


        last_obs = self.unwrapped.last_obs_maps

        if len(last_obs) and self.record_env_info:
            if 'collisions' in last_obs[0]:
                self.number_of_collisions += sum([1 if x['collisions'] else 0 for x in last_obs])
            if 'agents_velocity' in last_obs[0]:
                self.velocity_changes += sum([abs(x['agents_velocity'][0:2] - x['agents_velocity'][3:5]).sum() for x in last_obs])

        return res

    def reset(self, seed=None, return_info=False, options=None):
        if not self.tbx_writer:
            self.tbx_writer = get_tboard_writer(self.tbx_log)

            if self.record_video:
                image_topic = "/rviz1/camera1/image"
                rospy.Subscriber(image_topic, Image, self.image_callback)

        if len(self.video) > 0 and self.record_video:
            self.tbx_writer.add_video(f'eval_{self.video_name}' if self.unwrapped.in_eval else self.video_name, np.stack([np.stack(self.video)]), self.episode_count, fps=self.fps)

        if self.record_env_info and (self.episode_count > 0 and not self.unwrapped.in_eval) or (self.eval_episode_count > 0 and self.unwrapped.in_eval):

            if self.unwrapped.in_eval:
                title = 'eval_env_info'
                episode_count = self.eval_episode_count
            else:
                title = 'env_info'
                episode_count = self.episode_count

            self.tbx_writer.add_scalars(title, {
                'number_of_collisions': self.number_of_collisions,
                'full_successes': all([x.get('success', 0) for x in self.unwrapped.last_reward_maps]),
                'successful_agents': sum([1 if x.get('success') else 0 for x in self.unwrapped.last_reward_maps]),
                'velocity_changes': int(self.velocity_changes)
            }, episode_count)

        print({
                'number_of_collisions': self.number_of_collisions,
                'full_successes': all([x.get('success', 0) for x in self.unwrapped.last_reward_maps]),
                'successful_agents': sum([1 if x.get('success') else 0 for x in self.unwrapped.last_reward_maps]),
                'velocity_changes': int(self.velocity_changes)
            })
        self.number_of_collisions = 0
        self.velocity_changes = 0

        if self.unwrapped.in_eval:
            self.eval_episode_count += 1
            self.eval_step_count = 0
        else:
            self.episode_count += 1
            self.step_count = 0

        self.video = []

        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def seed(self, seed=None):
        pass
