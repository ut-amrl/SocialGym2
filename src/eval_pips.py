import gym
import sys
from random import seed
#  from stable_baselines3.common.policies import MlpPolicy
from ros_social_gym import RosSocialEnv

# The algorithms require a vectorized environment to run
if (len(sys.argv) < 2):
    print("Requires the path to launch file to evaluate.")
    exit()

launch = sys.argv[1]
env = RosSocialEnv(0, launch)
seed(1123)
model = None
#  obs = env.reset()

numScenarios = 4000
resetCount = 0
while resetCount < numScenarios:
    obs, rewards, dones, info = env.PipsStep()
    resetCount = int(info["resetCount"])
    if (dones):
        env.reset()
