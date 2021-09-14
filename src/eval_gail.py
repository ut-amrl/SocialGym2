import gym
import sys
from random import seed
#  from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
#  import pandas as pdj
from ros_social_gym import RosSocialEnv

from imitation.algorithms import adversarial, bc
from imitation.policies import serialize

if (len(sys.argv) < 2):
    print("Requires the path to the model to evaluate.")
    exit()

modelPath = sys.argv[1]
env = DummyVecEnv([lambda: RosSocialEnv('1', 1, "config/gym_gen/launch.launch")])
seed(1123)
model = serialize.load_policy("ppo", modelPath, env)

numScenarios = 2000
resetCount = 0
action = [0]
obs, rewards, dones, info = env.step(action)
while resetCount < numScenarios:
    action = [0]
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    resetCount = int(info[0]["resetCount"])
