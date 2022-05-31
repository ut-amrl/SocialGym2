import gym
import sys
from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from ros_social_gym import RosSocialEnv

env = DummyVecEnv([lambda: RosSocialEnv('1', 1, "config/gym_gen/launch.launch")])

numScenarios = 1000
resetCount = 0
action = [0]
obs, rewards, dones, info = env.step(action)
while resetCount < numScenarios:
    action = [0]
    obs, rewards, dones, info = env.step(action)
    resetCount = int(info[0]["resetCount"])
