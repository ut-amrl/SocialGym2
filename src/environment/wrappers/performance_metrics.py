import gym
from gym import Wrapper, Env
from pathlib import Path
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWraper
import time
from src.environment.utils.utils import get_tboard_writer

GymObs = Union[Tuple, Dict, np.ndarray, int]
import sys


class PerformanceMetricWrapper(BaseParallelWraper):
    """
    Limit the maximum number of steps per episode.

    :param env: Gym environment that will be wrapped
    :param tbx_log: The TBX Log name/path that you want to store performance metrics for.
    :param out_file: Json file that will be used to store raw output.
    """

    def __init__(self, env: gym.Env, tbx_log: str = None, out_file: Path = None):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)
        self.tbx_log = tbx_log
        self.tbx_writer = None
        self.perf_out = out_file

        self.episode_steps = 0
        self.episode_count = 0
        self.episode_time_start = None
        self.step_time_start = None
        self.step_time_deltas = []

    def reset(self, seed=None, return_info=False, options=None):
        _time = time.time()
        if self.episode_steps > 0:
            total_episode_time = _time - self.episode_time_start
            episode_time_normed = total_episode_time / self.episode_steps
            avg_step_time = sum(self.step_time_deltas) / self.episode_steps

            if not self.tbx_writer and self.tbx_log:
                self.tbx_writer = get_tboard_writer(self.tbx_log)
            if self.tbx_log:
                self.tbx_writer.add_scalars("Performance_Metrics", {
                    'total_episode_time': total_episode_time,
                    'episode_time_normed': episode_time_normed,
                    'avg_step_time': avg_step_time
                }, self.episode_count)

        self.episode_time_start = None
        self.step_time_deltas = []
        self.step_time_start = None
        self.episode_steps = 0
        self.episode_count += 1
        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        _time = time.time()
        if self.episode_steps == 0:
            # sys.activate_stack_trampoline("perf")
            self.episode_time_start = _time
            self.step_time_start = _time
            # sys.deactivate_stack_trampoline()
        else:
            self.step_time_deltas.append(_time - self.step_time_start)
            self.step_time_start = _time

        obs, reward, done, infos = self.env.step(action)
        self.agents = self.unwrapped.agents

        self.episode_steps += 1

        # return obs, reward, done, truncs, infos
        return obs, reward, done, infos

    def seed(self, seed=None):
        pass
