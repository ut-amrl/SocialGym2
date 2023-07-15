import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from tqdm import tqdm
from pettingzoo.utils.wrappers import BaseParallelWrapper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class ProgressBarWrapper(BaseParallelWrapper):
    """
    Limit the maximum number of steps per episode.

    :param env: Gym environment that will be wrapped
    :param total_steps: Total steps to train
    """

    def __init__(self, env: gym.Env, total_steps: int, iter_update: int = 100):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)
        self.total_steps = total_steps
        self.iter_update = iter_update
        self.training_timesteps = 0

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        in_eval = self.unwrapped.in_eval
        if not in_eval:
            self.training_timesteps += 1
            if self.training_timesteps % self.iter_update == 0:
                print(f"[TRAINING PROGRESS]: {self.training_timesteps}  /  {self.total_steps}")


        return self.env.step(action)

    def seed(self, seed=None):
        pass
