import torch as th
import json
from random import seed
import stable_baselines3 as sb3
import sb3_contrib as sb3c

from stable_baselines3 import PPO
from sb3_contrib import RecurrentPPO
from pettingzoo.test import parallel_test, parallel_api_test
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize, VecMonitor
from stable_baselines3.common.monitor import Monitor
from src.environment.callbacks.callbacks import EvalCallback, OptunaCallback
import numpy as np
from typing import Dict

import supersuit as ss
from functools import partial

from src.environment import ManualZoneEnv
from src.environment.rewards import Rewarder, Success, ExistencePenalty, \
  Collisions, LinearWeightScheduler, \
  GoalDistanceChange
from src.environment.rewards.types.manual_zone import EnforcedOrder
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, OthersPoses, OthersVelocities, CollisionObservation, OtherAgentObservables
from src.environment.observations.types.manual_zone import AgentInZone, AgentZoneCurrentOrder, AgentZonePriorityOrder
from src.environment.wrappers import NewScenarioWrapper, TensorboardWriter, EntropyEpisodeEnder, CollisionEpisodeEnder, \
  RewardStripper, TimeLimitWrapper
from src.environment.extractors import LSTMAgentObs

from src.environment.scenarios.common_scenarios import exp1_train_scenario, exp2_train_scenario
from src.environment.scenarios import CycleScenario
from src.environment.utils.utils import DATA_FOLDER
from src.environment.utils.evaluate_policy import evaluate_policy
import datetime

seed(1)


class kinds:
  sacadrl = 'SACADRL'
  ao = 'AO'
  eo = 'EO'


def run(
        num_agents: int = 2,
        train_length: int = 100_000,
        eval_frequency: int = 25_000,
        intermediate_eval_trials: int = 25,
        ending_eval_trials: int = 100,
        ending_eval_with_best: bool = True,
        device: str = 'cuda:0',
        partially_observable: bool = False,
        train: bool = True,
        eval: bool = True,

        monitor: bool = False,
        local: bool = False,

        existence_penalty: int = 1,
        success_reward: int = 100,

        collision_penalty: int = 10,
        starting_collision_penalty: int = 0,
        collision_penalty_scale_duration: int = 0,

        enforced_order_reward: int = 25,
        enforced_order_track_exit: bool = False,
        enforced_order_penalty_for_incorrect_order: bool = True,

        goal_distance_reward: bool = True,
        goal_distance_reward_clip: bool = False,

        collision_ender: bool = False,

        reward_stripper: bool = False,

        timelimit: bool = False,
        timelimit_threshold: int = 2000,

        entropy_ender: bool = True,
        entropy_max_distance: float = 0.25,
        entropy_max_timesteps: int = 100,
        entropy_reward: bool = True,
        entropy_constant_penalty: int = None,
        entropy_constant_penalty_only_those_that_did_not_finish: bool = True,
        entropy_reward_multiplier: float = 100_000.,
        entropy_multiply_negative_rewards_only: bool = False,

        collision_obs: bool = True,
        agent_goal_distance_obs: bool = True,
        agent_pose_obs: bool = True,
        agent_pose_ignore_theta: bool = True,
        agent_velocity_obs: bool = False,
        agent_velocity_ignore_theta: bool = True,
        other_poses_obs: bool = True,
        other_poses_ignore_theta: bool = True,
        other_poses_actual_positions: bool = False,
        other_velocities_obs: bool = False,
        other_velocities_ignore_theta: bool = True,

        policy_algo_sb3_contrib: bool = False,
        policy_algo_name: str = "PPO",
        policy_name: str = "MlpPolicy",
        policy_algo_kwargs=None,

        debug: bool = False,
        experiment_name: str = 'exp2',


        run_name: str = None,
        continue_from: str = None,

        run_type: str = kinds.sacadrl
):
  if run_name is None:
    utc_datetime = datetime.datetime.utcnow()
    formated_string = utc_datetime.strftime("%Y-%m-%d-%H%MZ")
    run_name = f'run__{formated_string}'

  exp_folder = DATA_FOLDER / run_name
  exp_folder.mkdir(exist_ok=True, parents=True)

  exp_eval_report = exp_folder / f'ending_eval_report__{run_type}.json'

  exp_checkpoint_folder = exp_folder / f'checkpoints__{run_type}'
  exp_tensorboard_folder = exp_folder / f'tensorboard__{run_type}'

  exp_checkpoint_folder.mkdir(exist_ok=True, parents=True)
  exp_tensorboard_folder.mkdir(exist_ok=True, parents=True)

  if policy_algo_kwargs is None:
    policy_algo_kwargs = {'verbose': 3, 'device': device, 'n_steps': 512 * num_agents, 'tensorboard_log': str(exp_tensorboard_folder)}
  if 'device' not in policy_algo_kwargs:
    policy_algo_kwargs['device'] = device
  if 'tensorboard_log' not in policy_algo_kwargs:
    policy_algo_kwargs['tensorboard_log'] = exp_tensorboard_folder
  if 'verbose' not in policy_algo_kwargs:
    policy_algo_kwargs['verbose'] = 3

  if experiment_name == 'exp1':
    scenario = exp1_train_scenario(level='easy', partially_observable=partially_observable, config_runner=True if not monitor and not local else False, all_config=monitor and not local)
  elif experiment_name == 'exp2':
    scenario = exp2_train_scenario(level='easy', partially_observable=partially_observable,
                                 config_runner=True if not monitor and not local else False, all_config=monitor and not local)

  observations = []

  if agent_goal_distance_obs:
    observations.append(AgentsGoalDistance(history_length=2))
  if agent_pose_obs:
    observations.append(AgentsPose(ignore_theta=agent_pose_ignore_theta))
  if agent_velocity_obs:
    observations.append(AgentsVelocity(ignore_theta=agent_velocity_ignore_theta, history_length=2))
  if collision_obs:
    observations.append(CollisionObservation())
  observations.append(
    SuccessObservation()
  )

  observations.append(OtherAgentObservables(
    pos_x=other_poses_obs,
    pos_y=other_poses_obs,
    pos_theta=other_poses_obs and not other_poses_ignore_theta,
    vel_x=other_velocities_obs,
    vel_y=other_velocities_obs,
    vel_theta=other_poses_obs and not other_velocities_ignore_theta
  ))

  if run_type != kinds.sacadrl:
    observations.extend([
      AgentZonePriorityOrder(),
      AgentZoneCurrentOrder(),
      AgentInZone()
    ])

  rewards = []

  if existence_penalty > 0:
    rewards.append(ExistencePenalty(existence_penalty))
  if success_reward > 0:
    rewards.append(Success(weight=success_reward))
  if collision_penalty > 0:
    if collision_penalty_scale_duration > 0:
      rewards.append(LinearWeightScheduler(Collisions(weight=1.0), min_weight=starting_collision_penalty, max_weight=collision_penalty, duration=collision_penalty_scale_duration))
    else:
      rewards.append(Collisions(weight=collision_penalty))
  if goal_distance_reward:
    rewards.append(GoalDistanceChange(clip=goal_distance_reward_clip))
  if enforced_order_reward > 0 and run_type != kinds.sacadrl:
    rewards.append(EnforcedOrder(
      weight=enforced_order_reward,
      on_enter=True,
      on_exit=enforced_order_track_exit,
      allow_any_order=run_type == kinds.ao,
      incorrect_penalty=enforced_order_penalty_for_incorrect_order and run_type == kinds.eo,
    ))

  observer = Observer(observations)
  rewarder = Rewarder(rewards)

  ENV_CLASS = partial(ManualZoneEnv, 7, 11, 1.5)

  env = ENV_CLASS(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=num_agents, debug=debug)

  if entropy_ender:
    env = EntropyEpisodeEnder(
      env,
      timestep_threshold=entropy_max_timesteps,
      distance_delta=entropy_max_distance,
      negative_multiplier_only=entropy_multiply_negative_rewards_only,
      constant_reward_on_end=entropy_constant_penalty,
      only_those_that_did_not_finish=entropy_constant_penalty_only_those_that_did_not_finish,
      reward_multiplier=entropy_reward_multiplier,
      update_rewards=entropy_reward
    )
  if collision_ender:
    env = CollisionEpisodeEnder(env)
  if timelimit:
    env = TimeLimitWrapper(env, max_steps=timelimit_threshold)
  if reward_stripper:
    env = RewardStripper(env)


  env = NewScenarioWrapper(env, new_scenario_episode_frequency=1, plans=[[0, 2], [1000, 3], [2000, 4]])

  env = TensorboardWriter(
    env,
    tbx_log=f'{run_name}/tensorboard__{run_type}',
    record_video=False,
    record_rewards=True,
    video_sample_rate=1,
    step_sample_rate=1
  )

  env = ss.black_death_v3(env)
  env = ss.pad_observations_v0(env)
  env = ss.pad_action_space_v0(env)
  env = ss.pettingzoo_env_to_vec_env_v1(env)
  env.black_death = True

  env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')

  env = VecNormalize(env, norm_reward=True, norm_obs=True, clip_obs=10.)
  env = VecMonitor(env)

  policy_algo_kwargs['policy_kwargs'] = {"features_extractor_class": LSTMAgentObs, "features_extractor_kwargs": dict(observer=observer)}

  if policy_algo_sb3_contrib:
    model = getattr(sb3c, policy_algo_name)(policy_name, env, **policy_algo_kwargs)
  else:
    model = getattr(sb3, policy_algo_name)(policy_name, env, **policy_algo_kwargs)

  if continue_from:
    model = model.load(DATA_FOLDER / continue_from, env)

  eval_callback = EvalCallback(
    eval_env=env,
    n_eval_episodes=intermediate_eval_trials,
    eval_freq=eval_frequency,
    best_model_save_path=str(exp_checkpoint_folder),
    eval_report_file=exp_eval_report,
    number_of_agents=list(range(7))[2:]
  )

  if train:
    model.learn(
      train_length * num_agents,
      callback=eval_callback if eval_frequency > 0 else None,
      tb_log_name=str(exp_tensorboard_folder)
    )

    model.save(exp_checkpoint_folder / 'last')

    if eval_frequency > 0 and ending_eval_with_best:
      model = model.load(exp_checkpoint_folder / 'best_model', env=env, device=device)
    else:
      model = model.load(exp_checkpoint_folder / 'last', env=env, device=device)

  if eval:
    eval_report = {}

    all_rates = []

    for a in range(2, num_agents + 1):
      env.unwrapped.vec_envs[0].par_env.unwrapped.in_eval = True
      env.unwrapped.vec_envs[0].par_env.unwrapped.curr_num_agents = a
      env.unwrapped.vec_envs[0].par_env.unwrapped.new_scenario(num_agents=a)
      env.unwrapped.vec_envs[0].par_env.unwrapped.reset()

      episode_rewards, episode_lengths, success_rate = evaluate_policy(
        model,
        env,
        n_eval_episodes=ending_eval_trials,
        deterministic=True,
        return_episode_rewards=True,
      )


      eval_report[str(a)] = success_rate
      all_rates.append(success_rate)
    eval_report['total'] = sum(all_rates) / len(all_rates)

    with exp_eval_report.open('w') as f:
      json.dump(eval_report, f)

if __name__ == "__main__":
  import argparse
  from pathlib import Path

  argparser = argparse.ArgumentParser()
  argparser.add_argument('--config', '-c', type=str, help='Config file with options for this file.')

  args = argparser.parse_args()

  config: Path = Path(args.config)

  with config.open('r') as f:
    run_arguments = json.load(f)

  assert run_arguments is not None, 'config load error'

  run(
    **run_arguments
  )
