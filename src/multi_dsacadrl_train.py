import torch as th
import json
from random import seed
from stable_baselines3 import PPO
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize, VecMonitor
from stable_baselines3.common.monitor import Monitor
from src.environment.callbacks.callbacks import EvalCallback, OptunaCallback
import numpy as np

import supersuit as ss
from functools import partial

from src.environment import ManualZoneEnv
from src.environment.rewards import Rewarder, Success, ExistencePenalty, \
  Collisions, LinearWeightScheduler, \
  GoalDistanceChange
from src.environment.rewards.types.manual_zone import EnforcedOrder
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, OthersPoses, OthersVelocities,\
  CollisionObservation
from src.environment.observations.types.manual_zone import AgentInZone, AgentZoneCurrentOrder, AgentZonePriorityOrder
from src.environment.wrappers import NewScenarioWrapper, TensorboardWriter, EntropyEpisodeEnder

from src.environment.scenarios.common_scenarios import exp1_train_scenario
from src.environment.utils.utils import DATA_FOLDER
from src.environment.utils.evaluate_policy import evaluate_policy

seed(1)


exp_folder = DATA_FOLDER / '11_12_22_exps_mlplstm'
exp_report = exp_folder / 'report.json'
exp_checkpoint_folder = exp_folder / 'checkpoints'
exp_tensorboard_folder = exp_folder / 'tensorboard'

report = {}

if exp_report.exists():
  with exp_report.open('r') as f:
    report = json.load(f)


class kinds:
  sacadrl = 'SACADRL'
  ao = 'AO'
  eo = 'EO'


def objective(agents=2, kind=kinds.sacadrl):
  global exp_folder, exp_report, exp_checkpoint_folder, exp_tensorboard_folder, report, trial_num

  NUMBER_OF_AGENTS = agents
  TRAIN_LENGTH = 100_000
  EVAL_FREQ = 101_000
  INTERMEDIATE_EVAL_TRIALS = 25
  ENDING_EVAL_TRIALS = 100
  scenario = exp1_train_scenario(level='easy', partially_observable=False)

  relative_other_poses = False #trial.suggest_categorical("relative_other_poses", [True, False])

  existence_penalty = 1 #trial.suggest_float("existence_penalty", 0.5, 1.0, step=0.1)
  success_reward = 100  # trial.suggest_categorical("success_reward", [1, 10, 100])

  collisions_max_penalty = 10 #trial.suggest_int("collision_max_penalty", 30, 100, step=10)
  collision_min_penalty = 0.01 #trial.suggest_float("collision_min_penalty", 0.3, 1.0, step=0.05)
  collision_scale_duration = 100 #trial.suggest_int("collision_scale_duration", 20, 50, step=10)

  enforced_order_reward = 25 #trial.suggest_int("enforced_order_reward", 10, 100, step=10)
  enforced_order_reward_on_exit = True  # trial.suggest_categorical("enforced_order_reward_on_exit", [True, False])
  enforced_order_reward_incorrect_penalty = True  # trial.suggest_categorical("enforced_order_reward_incorrect_penalty", [True, False])

  entropy_max_distance = 0.5 #trial.suggest_float("entropy_max_distance", 0.3, 1, step=0.1)
  entropy_max_timesteps = 100 #trial.suggest_int("entropy_max_timesteps", 60, 120, step=10)
  # entropy_constant_penalty = trial.suggest_categorical("entropy_constant_penalty", [True, False])
  # entropy_multiply_negative_rewards_only = trial.suggest_categorical("entropy_multiply_negative_rewards", [True, False])
  entropy_only_penalize_agents_that_did_not_finish = True

  entropy_max_timesteps *= NUMBER_OF_AGENTS

  if kind == kinds.sacadrl:
    observations = [
      AgentsGoalDistance(history_length=2),
      AgentsPose(ignore_theta=True),
      # AgentsVelocity(history_length=2, ignore_theta=True),
      OthersPoses(actuals=False, ignore_theta=True),
      # OthersVelocities(ignore_theta=True),
      SuccessObservation(),
      CollisionObservation(),
    ]
    rewards = [
      ExistencePenalty(existence_penalty),
      Success(weight=success_reward),
      Collisions(weight=collisions_max_penalty),
      # LinearWeightScheduler(Collisions(weight=1.0), min_weight=collision_min_penalty, max_weight=collisions_max_penalty,
      #                       duration=collision_scale_duration),
      GoalDistanceChange(),
    ]

  elif kind == kinds.ao:
    observations = [
      AgentsGoalDistance(history_length=2),
      AgentsPose(ignore_theta=True),
      # AgentsVelocity(history_length=2, ignore_theta=True),
      OthersPoses(actuals=False, ignore_theta=True),
      # OthersVelocities(ignore_theta=True),
      SuccessObservation(),
      CollisionObservation(),
      AgentZonePriorityOrder(),
      AgentZoneCurrentOrder(),
      AgentInZone(),
    ]
    rewards = [
      ExistencePenalty(existence_penalty),
      Success(weight=success_reward),
      Collisions(weight=collisions_max_penalty),
      # LinearWeightScheduler(Collisions(weight=1.0), min_weight=collision_min_penalty, max_weight=collisions_max_penalty,
      #                       duration=collision_scale_duration),
      GoalDistanceChange(),
      EnforcedOrder(weight=enforced_order_reward, on_enter=True, on_exit=False,
                    continuous=False, weak_out_of_zone=False,
                    allow_any_order=True, incorrect_penalty=False)
    ]
  else:
    observations = [
      AgentsGoalDistance(history_length=2),
      AgentsPose(ignore_theta=True),
      # AgentsVelocity(history_length=2, ignore_theta=True),
      OthersPoses(actuals=False, ignore_theta=True),
      # OthersVelocities(ignore_theta=True),
      SuccessObservation(),
      CollisionObservation(),
      AgentZonePriorityOrder(),
      AgentZoneCurrentOrder(),
      AgentInZone(),
    ]
    rewards = [
      ExistencePenalty(existence_penalty),
      Success(weight=success_reward),
      Collisions(weight=collisions_max_penalty),
      # LinearWeightScheduler(Collisions(weight=1.0), min_weight=collision_min_penalty, max_weight=collisions_max_penalty,
      #                       duration=collision_scale_duration),
      GoalDistanceChange(),
      EnforcedOrder(weight=enforced_order_reward, on_enter=True, on_exit=False,
                    continuous=False, weak_out_of_zone=False,
                    allow_any_order=False, incorrect_penalty=True)
    ]

  observer = Observer(observations)
  rewarder = Rewarder(rewards)

  MODEL_NAME = f'{exp_folder.name}/{exp_tensorboard_folder.name}/agents_{str(agents)}__{kind}'
  TBX_LOG = MODEL_NAME
  CHECKPOINT_PATH = exp_checkpoint_folder / f'agents_{str(agents)}__{kind}'

  CHECKPOINT_PATH.mkdir(parents=True, exist_ok=True)

  # ENV_CLASS = RosSocialEnv
  ENV_CLASS = partial(ManualZoneEnv, 7, 11, 1.5)

  env = ENV_CLASS(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=NUMBER_OF_AGENTS)

  env = EntropyEpisodeEnder(env, timestep_threshold=entropy_max_timesteps, distance_delta=entropy_max_distance, negative_multiplier_only=False, constant_reward_on_end=None, only_those_that_did_not_finish=False, reward_multiplier=100_000, update_rewards=True)
  env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)

  env = TensorboardWriter(env, tbx_log=TBX_LOG, record_video=False, record_rewards=True, video_sample_rate=1,
                            step_sample_rate=1)


  env = ss.pettingzoo_env_to_vec_env_v1(env)

  env = ss.concat_vec_envs_v1(env, 1, num_cpus=10, base_class='stable_baselines3')

  env = VecNormalize(env, norm_reward=True, norm_obs=True, clip_obs=10.)
  env = VecMonitor(env)


  # policy_kwargs = dict(activation_fn=th.nn.ReLU,
  #                      net_arch=[dict(pi=[256, 256], vf=[256, 256])])

  # model = PPO("MlpPolicy", env, verbose=3, device='cuda:1', n_steps=256 * NUMBER_OF_AGENTS)#policy_kwargs=policy_kwargs)
  model = RecurrentPPO("MlpLstmPolicy", env, verbose=3, device='cuda:1', n_steps=1024 * NUMBER_OF_AGENTS, tensorboard_log=str(DATA_FOLDER / TBX_LOG))#, policy_kwargs=policy_kwargs)

  # model = model.load(f"{DATA_FOLDER / f'11_9_22_exps/checkpoints/agents_{str(agents)}__{kind}/last'}", policy_kwargs=policy_kwargs, env=env, n_steps=2048 * NUMBER_OF_AGENTS, device='cuda:1')

  # optuna_callback = OptunaCallback(
  #   trial=trial
  # )

  checkpoint_callback = CheckpointCallback(save_freq=100_000 * NUMBER_OF_AGENTS, save_path=CHECKPOINT_PATH)
  eval_callback = EvalCallback(
    eval_env=env,
    n_eval_episodes=INTERMEDIATE_EVAL_TRIALS,
    eval_freq=EVAL_FREQ,
    best_model_save_path=CHECKPOINT_PATH,
    # callback_after_eval=optuna_callback
  )
  callback = CallbackList([checkpoint_callback, eval_callback])

  with (CHECKPOINT_PATH / 'config.json').open('w') as f:
    json.dump({
      'kind': kind,
      'agents': agents,
      'relative_other_poses': relative_other_poses,
      'existence_penalty': existence_penalty,
      'success_reward': success_reward,
      'collisions_max_penalty': collisions_max_penalty,
      'collision_min_penalty': collision_min_penalty,
      'collision_scale_duration': collision_scale_duration,
      'enforced_order_reward': enforced_order_reward,
      'enforced_order_reward_on_exit': enforced_order_reward_on_exit,
      'enforced_order_reward_incorrect_penalty': enforced_order_reward_incorrect_penalty,
      'entropy_max_distance': entropy_max_distance,
      'entropy_max_timesteps': entropy_max_timesteps,
    }, f)

  model.learn(TRAIN_LENGTH * NUMBER_OF_AGENTS, callback=callback, tb_log_name=str(DATA_FOLDER / TBX_LOG))
  model.save(CHECKPOINT_PATH / 'last')
  #
  # if (CHECKPOINT_PATH / 'best_model').exists():
  #   model = model.load(CHECKPOINT_PATH / 'best_model', env=env, device='cuda:1')
  # else:
  model = model.load(CHECKPOINT_PATH / 'last', env=env, device='cuda:1')

  episode_rewards, episode_lengths, success_rate = evaluate_policy(
    model,
    model.get_env(),
    n_eval_episodes=ENDING_EVAL_TRIALS,
    deterministic=True,
    return_episode_rewards=True,

  )

  report[f'agents_{str(agents)}__{kind}'] = success_rate
  with exp_report.open('w') as f:
    json.dump(report, f)

  #trial_num += 1


if __name__ == "__main__":

  # for i in range(5, 1, -1):
  for i in range(2, 6, 1):
    print(f"+===+ Agents = {i} +===+")
    print("=== SACADRL ===")
    objective(i, kinds.sacadrl)
    print("=== AO ===")
    objective(i, kinds.ao)
    print("=== EO ===")
    objective(i, kinds.eo)
