from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import get_tboard_writer, filter_stdout, LogFilter
from src.environment.rewards import Rewarder, Success, GoalDistance
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation
from src.environment.observations.common_observations import dsacadrl_observations
from src.environment.wrappers.new_scenario_wrapper import NewScenarioWrapper
from src.environment.wrappers.time_limit import TimeLimitWrapper


tbx_writer, tbx_logdir = get_tboard_writer('dqn_sacadrl')

observer = Observer([AgentsPose(), AgentsGoalDistance(1), SuccessObservation()])
rewarder = Rewarder([GoalDistance(), Success()], tbx_writer=tbx_writer)

EPISODE_LENGTH = 2_000

# The algorithms require a vectorized environment to run
env = TimeLimitWrapper(NewScenarioWrapper(
  RosSocialEnv('1', 1, "config/gym_gen/launch.launch", observer, rewarder, 'closed/door/t1', 0, tbx_writer=tbx_writer),
  new_scenario_episode_frequency=1
), max_steps=EPISODE_LENGTH)
env = DummyVecEnv([lambda: Monitor(env)])

seed(1)

GYM_TBX = True
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log=tbx_logdir if GYM_TBX else None, learning_starts=EPISODE_LENGTH * 1)

model.learn(EPISODE_LENGTH * 250, eval_env=env, eval_freq=EPISODE_LENGTH * 5)

# model = DQN("MlpPolicy", env, verbose=1)

# count = 0
# upper = 1000
# eval_freq = 10


# while(count < upper):
#
#   if count % eval_freq == 0:
#     # evaluate_policy(model, env, n_eval_episodes=3, deterministic=True)
#     model.learn(total_timesteps=2000, callback=EvalCallback(env, eval_freq=2000))
#   else:
#     model.learn(total_timesteps=2000)

  # Save the agent
  # model.save("data/dqn_" + str(count))
  # count += 1
