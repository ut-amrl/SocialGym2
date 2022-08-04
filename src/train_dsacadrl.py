from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import get_tboard_writer
from src.environment.rewards import Rewarder
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer
from src.environment.observations.common_observations import dsacadrl_observations

import torch as th
import datetime
from pathlib import Path
import shutil


policy_kwargs = dict(net_arch=[150, 100, 100], activation_fn=th.nn.ReLU)

tbx_writer = get_tboard_writer('dqn_sacadrl')

observer = Observer(dsacadrl_observations())
rewarder = Rewarder(registered_rewards=dsacadrl_rewards(), tbx_writer=tbx_writer)

# The algorithms require a vectorized environment to run
env = DummyVecEnv([lambda: RosSocialEnv('1', 20, "config/gym_gen/launch.launch", observer, rewarder, 'open_t1', 1)])
seed(1)
model = DQN("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs)

count = 0
upper = 1000
while(count < upper):
  model.learn(total_timesteps=2000)
  # Save the agent
  # model.save("data/dqn_" + str(count))
  count += 1
