from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.rewards import GoalDistance, Success
from src.environment.observations import AgentPose, AgentVelocity
import torch as th

policy_kwargs = dict(net_arch=[150, 100, 100], activation_fn=th.nn.ReLU)

observations = [AgentPose(), AgentVelocity()]
rewards = [GoalDistance(), Success()]

# The algorithms require a vectorized environment to run
env = DummyVecEnv([lambda: RosSocialEnv('1', 20, "config/gym_gen/launch.launch", observations, rewards)])
seed(1)
model = DQN("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, learning_starts=0)

count = 0
upper = 1000
while(count < upper):
  model.learn(total_timesteps=1)
  # Save the agent
  # model.save("data/dqn_" + str(count))
  count += 1
