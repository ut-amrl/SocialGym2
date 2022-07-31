import sys
from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from src.environment.ros_social_gym import RosSocialEnv

from imitation.algorithms import bc

if (len(sys.argv) < 2):
    print("Requires the path to the model to evaluate.")
    exit()

env = DummyVecEnv([lambda: RosSocialEnv('1', 1, "config/gym_gen/launch.launch")])
seed(1123)
modelPath = sys.argv[1]
model = bc.reconstruct_policy(modelPath)

numScenarios = 2000
resetCount = 0
action = [0]
obs, rewards, dones, info = env.step(action)
while resetCount < numScenarios:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    resetCount = int(info[0]["resetCount"])
