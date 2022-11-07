from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN, PPO
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from src.environment.callbacks.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from pathlib import Path
import supersuit as ss
from pettingzoo.utils import wrappers
from functools import partial

from src.environment import RosSocialEnv, ManualZoneEnv
from src.environment.utils.utils import get_tboard_writer, filter_stdout, LogFilter
from src.environment.rewards import Rewarder, Success, GoalDistance, ExistencePenalty, SocialNormPass, \
  SocialNormOvertake, SocialNormCross, Collisions, VelocityControl, PreferredVelocity, LinearWeightScheduler, \
  GoalDistanceChange
from src.environment.rewards.types.manual_zone import EnforcedOrder
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, AgentsHeadingDirection, AgentsOthersDistance, AgentsPreferredVelocity, OthersPoses, OthersVelocities,\
  OthersHeadingDirection, CollisionObservation
from src.environment.observations.types.manual_zone import AgentInZone, AgentZoneCurrentOrder, AgentZonePriorityOrder
from src.environment.observations.common_observations import dsacadrl_observations
from src.environment.wrappers import NewScenarioWrapper, TimeLimitWrapper, TensorboardWriter, CollisionEpisodeEnder, EntropyEpisodeEnder

from src.environment.scenarios import GraphNavScenario, CycleScenario
from src.environment.scenarios.common_scenarios import closed_door_1__same_goals, closed_door_1__opposing_sides, elevator_loading, exp1_train_scenario
from src.environment.utils.utils import ROOT_FOLDER, DATA_FOLDER
from src.environment.utils.evaluate_policy import evaluate_policy

seed(1)


TRAIN = True
CONTINUE = False
EVAL = False
USE_TENSOR_BOARD = True


MODEL_NAME = '11_6_22_exps/5a_eo'
# MODEL_NAME = 'tmp'
TBX_LOG = MODEL_NAME if TRAIN else f'{MODEL_NAME}__eval'
CHECKPOINT_PATH = DATA_FOLDER / 'checkpoints' / MODEL_NAME


observations = [
  AgentsGoalDistance(history_length=2),
  AgentsPose(),
  AgentsVelocity(history_length=2),
  # AgentsHeadingDirection(),
  # AgentsOthersDistance(),
  # AgentsPreferredVelocity(preferred_velocity=1.0),
  OthersPoses(actuals=True),
  OthersVelocities(),
  # OthersHeadingDirection(),
  SuccessObservation(),
  CollisionObservation(),
  AgentZonePriorityOrder(),
  AgentZoneCurrentOrder(),
  AgentInZone(),
]

rewards = [
  ExistencePenalty(1),
  Success(weight=100),
  # SocialNormCross(weight=33),
  # SocialNormOvertake(weight=33),
  # SocialNormPass(weight=33),
  LinearWeightScheduler(Collisions(weight=10.00), min_weight=0.01, max_weight=1.0, duration=10),
  # VelocityControl(weight=.1),
  # PreferredVelocity(),
  # GoalDistance(),
  GoalDistanceChange(),
  EnforcedOrder(weight=50, on_enter=True, on_exit=False, continuous=False, weak_out_of_zone=False, allow_any_order=False, incorrect_penalty=True)
]

observer = Observer(observations)
rewarder = Rewarder(rewards)

NUMBER_OF_AGENTS = 5
EPISODE_LENGTH = 5_000
TRAIN_LENGTH = 250_000

# scenario = GraphNavScenario('elevator/t1')
# scenario = closed_door_1__same_goals('t1')
# scenario = elevator_loading()
# scenario = CycleScenario('exp1/train/easy')
scenario = exp1_train_scenario(level='easy', partially_observable=False)

#ENV_CLASS = RosSocialEnv
ENV_CLASS = partial(ManualZoneEnv, 7, 11, 1.5)

env = ENV_CLASS(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=NUMBER_OF_AGENTS)
env = EntropyEpisodeEnder(env, distance_delta=0.02, negative_multiplier_only=False, constant_reward_on_end=-100_000)
# env = CollisionEpisodeEnder(env)
# env = TimeLimitWrapper(env, max_steps=int(EPISODE_LENGTH))
env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)
if USE_TENSOR_BOARD:
  env = TensorboardWriter(env, tbx_log=TBX_LOG, record_video=False, record_rewards=True, video_sample_rate=1, step_sample_rate=1)

env = ss.pettingzoo_env_to_vec_env_v1(env)

# Multi-processing/threading is not supported atm.
env = ss.concat_vec_envs_v1(env, 1, num_cpus=10, base_class='stable_baselines3')


model = PPO("MlpPolicy", env, verbose=3, device='cuda:1')
if CONTINUE and (CHECKPOINT_PATH / 'last.zip').exists():
  model = model.load(CHECKPOINT_PATH / 'last', env=env, device='cuda:1')

# model = DQN("MlpPolicy", env, verbose=3, device='cuda:1')
# model = model.load(DATA_FOLDER / 'checkpoints' / 'baseline_v2_enforceorder_collision_pen2' / 'rl_model_900000_steps', env=env, device='cuda:1')

# TODO - fix this, the multi-agent should probably be an algorithm not an env wrapper (or maybe both)
# env.envs[0].env.env.env.model = model
checkpoint_callback = CheckpointCallback(save_freq=10_000 * NUMBER_OF_AGENTS, save_path=CHECKPOINT_PATH)
eval_callback = EvalCallback(
  eval_env=env,
  n_eval_episodes=10,
  eval_freq=10_000 * NUMBER_OF_AGENTS,
  best_model_save_path=CHECKPOINT_PATH
)

callback = CallbackList([checkpoint_callback, eval_callback])

if TRAIN:
  model.learn(TRAIN_LENGTH * NUMBER_OF_AGENTS, callback=callback)
  model.save(CHECKPOINT_PATH / 'last')

if EVAL:
  model = model.load(CHECKPOINT_PATH / 'best_model', env=env, device='cuda:1')
  # model = model.load(CHECKPOINT_PATH / 'rl_model_600000_steps.zip')

  evaluate_policy(model, model.get_env(), n_eval_episodes=100)


# model_file = Path("baselines")
# model_file.mkdir(parents=True, exist_ok=True)
#
# model.save(str(model_file / f'{MODEL_NAME}'))
