from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN, PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import get_tboard_writer, filter_stdout, LogFilter
from src.environment.rewards import Rewarder, Success, GoalDistance, ExistencePenalty, SocialNormPass, SocialNormOvertake, SocialNormCross, Collisions
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, AgentsHeadingDirection, AgentsOthersDistance, AgentsPreferredVelocity, OthersPoses, OthersVelocities,\
  OthersHeadingDirection, CollisionObservation
from src.environment.observations.common_observations import dsacadrl_observations
from src.environment.wrappers.new_scenario_wrapper import NewScenarioWrapper
from src.environment.wrappers.time_limit import TimeLimitWrapper
from src.environment.scenarios import GraphNavScenario
from src.environment.scenarios.common_scenarios import closed_door_1__same_goals, closed_door_1__opposing_sides

seed(1)

tbx_writer, tbx_logdir = get_tboard_writer('dqn_sacadrl')

observations = [
  AgentsGoalDistance(),
  AgentsPose(),
  AgentsVelocity(),
  AgentsHeadingDirection(),
  AgentsOthersDistance(),
  AgentsPreferredVelocity(preferred_velocity=1.0),
  OthersPoses(),
  OthersVelocities(),
  OthersHeadingDirection(),
  SuccessObservation(),
  CollisionObservation()
]

rewards = [
  ExistencePenalty(),
  Success(),
  SocialNormCross(),
  SocialNormOvertake(),
  SocialNormPass(),
  Collisions(weight=100)
]

observer = Observer(observations)
rewarder = Rewarder(rewards, tbx_writer=tbx_writer)

EPISODE_LENGTH = 2_000

scenario = closed_door_1__same_goals()

env = RosSocialEnv('1', 1, "config/gym_gen/launch.launch", observer, rewarder, scenario, 3, tbx_writer=tbx_writer)
env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)
env = TimeLimitWrapper(env, max_steps=EPISODE_LENGTH)
env = Monitor(env)
env = DummyVecEnv([lambda: env])


GYM_TBX = True
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tbx_logdir if GYM_TBX else None)

model.learn(EPISODE_LENGTH * 250, eval_env=env, eval_freq=EPISODE_LENGTH * 5)
