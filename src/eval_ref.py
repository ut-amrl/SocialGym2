from random import seed
from src.environment.ros_social_gym import RosSocialEnv

# The algorithms require a vectorized environment to run
env = RosSocialEnv('1', 1, 'config/gym_gen/ref_launch.launch')
seed(1123)

numScenarios = 2000
resetCount = 0
while resetCount < numScenarios:
    print("Reset Count: " + str(resetCount))
    obs, rewards, dones, info = env.PipsStep()
    resetCount = int(info["resetCount"])
    if (dones):
        env.reset()
