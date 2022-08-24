from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN, PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from pathlib import Path

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import get_tboard_writer, filter_stdout, LogFilter
from src.environment.rewards import Rewarder, Success, GoalDistance, ExistencePenalty, SocialNormPass, SocialNormOvertake, SocialNormCross, Collisions, VelocityControl, PreferredVelocity
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, AgentsHeadingDirection, AgentsOthersDistance, AgentsPreferredVelocity, OthersPoses, OthersVelocities,\
  OthersHeadingDirection, CollisionObservation
from src.environment.observations.common_observations import dsacadrl_observations
from src.environment.wrappers.new_scenario_wrapper import NewScenarioWrapper
from src.environment.wrappers.time_limit import TimeLimitWrapper
from src.environment.scenarios import GraphNavScenario
from src.environment.scenarios.common_scenarios import closed_door_1__same_goals, closed_door_1__opposing_sides, elevator_loading


seed(1)

tbx_writer, tbx_logdir = get_tboard_writer('dqn_sacadrl')

MODEL_NAME = "tmp"

observations = [
  AgentsGoalDistance(),
  AgentsPose(),
  AgentsVelocity(history_length=2),
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
  ExistencePenalty(0.1),
  Success(),
  SocialNormCross(weight=33),
  SocialNormOvertake(weight=33),
  SocialNormPass(weight=33),
  Collisions(weight=100),
  VelocityControl(weight=.1),
  PreferredVelocity()
]

observer = Observer(observations)
rewarder = Rewarder(rewards, tbx_writer=tbx_writer)

EPISODE_LENGTH = 2_000
TRAIN_LENGTH = 50

# scenario = GraphNavScenario('elevator/t1')
# scenario = closed_door_1__same_goals('t1')
scenario = elevator_loading()

env = RosSocialEnv('1', 1, "config/gym_gen/launch.launch", observer, rewarder, scenario, 3, tbx_writer=tbx_writer, record_video=True)
env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)
env = TimeLimitWrapper(env, max_steps=EPISODE_LENGTH)
env = Monitor(env)
env = DummyVecEnv([lambda: env])


GYM_TBX = True
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tbx_logdir if GYM_TBX else None)

model.learn(EPISODE_LENGTH * TRAIN_LENGTH, eval_env=env, eval_freq=EPISODE_LENGTH * 5, n_eval_episodes=5)

model_file = Path("baselines")
model_file.mkdir(parents=True, exist_ok=True)

model.save(str(model_file / f'{MODEL_NAME}'))
