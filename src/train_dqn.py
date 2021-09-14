import gym
from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
from ros_social_gym import RosSocialEnv

# The algorithms require a vectorized environment to run
env = DummyVecEnv([lambda: RosSocialEnv('1', 20, "config/gym_gen/launch.launch")])
seed(1)
model = DQN("MlpPolicy", env, verbose=0)

count = 0
upper = 1000
while(count < upper):
  model.learn(total_timesteps=2000)
  # Save the agent
  model.save("data/dqn_" + str(count))
  count += 1
