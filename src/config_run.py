import torch as th
import json
from random import seed
import stable_baselines3 as sb3
import sb3_contrib as sb3c
import sys

from stable_baselines3 import PPO
from sb3_contrib import RecurrentPPO
from pettingzoo.test import parallel_test, parallel_api_test
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize, VecMonitor
from stable_baselines3.common.monitor import Monitor
from src.environment.callbacks.callbacks import EvalCallback, OptunaCallback
import numpy as np
from typing import Dict, List, Union

import supersuit as ss
from functools import partial

from src.environment import ManualZoneEnv
from src.environment import RosSocialEnv
from src.environment.rewards import Rewarder, Success, ExistencePenalty, \
  Collisions, LinearWeightScheduler, \
  GoalDistanceChange
from src.environment.rewards.types.manual_zone import EnforcedOrder
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, OthersPoses, OthersVelocities, CollisionObservation, OtherAgentObservables
from src.environment.observations.types.manual_zone import AgentInZone, AgentZoneCurrentOrder, AgentZonePriorityOrder, \
  EnteringZone, ExitingZone, NumberOfAgentsEnteringZone, NumberOfAgentsExitingZone
from src.environment.wrappers import NewScenarioWrapper, TensorboardWriter, EntropyEpisodeEnder, CollisionEpisodeEnder, \
  RewardStripper, TimeLimitWrapper, ProgressBarWrapper, PerformanceMetricWrapper
from src.environment.extractors import LSTMAgentObs
from src.environment.visuals.nav_map_viz import NavMapViz

from src.environment.scenarios.common_scenarios import envs_door, envs_hallway, envs_intersection, envs_round_about, \
  envs_open
from src.environment.scenarios import CycleScenario
from src.environment.utils.utils import DATA_FOLDER
from src.environment.utils.evaluate_policy import evaluate_policy
import datetime
from src.environment.utils.utils import get_tboard_writer, ROOT_FOLDER
from tqdm import tqdm

seed(1)


class kinds:
  sacadrl = 'SACADRL'
  ao = 'AO'
  eo = 'EO'


def run(
        num_agents: Union[List[int], int] = 2,
        eval_num_agents: Union[List[int], int] = None,
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

        existence_penalty: float = 1,
        success_reward: float = 100,

        collision_penalty: float = 10,
        starting_collision_penalty: float = 0,
        collision_penalty_scale_duration: float = 0,

        enforced_order_reward: float = 25,
        enforced_order_track_exit: bool = False,
        enforced_order_penalty_for_incorrect_order: bool = True,
        enforced_order_weak_signal_out_of_zone: bool = False,
        penalty_for_multiple_agents_entering: bool = True,

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
        experiment_names: List[str] = 'envs_open',


        run_name: str = None,
        continue_from: str = None,

        run_type: str = kinds.sacadrl
):
  """
  :param num_agents: Int or List[Int] - Either the exact number of agents to use, or an array of the format [[episode #,
    num of agents], ...] which will alter the number of agents once that episode limit has been reached.  I.E. [[0, 3],
    [10, 4], [20, 5]] will initially train with 3 agents until episode 10 which will then change to 4 agents and then
    change to 5 once episode 20 is reached.

  :param eval_num_agents: Int or List[Int] - Either the number of agents to evaluate on or a list of a number of agents
    to evaluate on (if list, [1, 2], evaluations will run for the number of specified episodes for 1 agent and then
    agian using 2 agents).

  :param train_length: Int - Number of steps to train for.  (remember each agent requires a step, therefore 500 steps
    for training 5 agents means each agent will get 100 steps each.

  :param eval_frequency: Int - Interval for running intermediate evaluation (in steps).   0>= will effectively disable
    evaluation during training.

  :param intermediate_eval_trials: Int - How many episodes to evaluate on during intermediate evaluation.

  :param ending_eval_trials: Int - Number of episodes to evaluate for after training has completed.

  :param ending_eval_with_best: Bool - After training, load up the best model seen during intermediate eval (if False,
    the most recent checkpoint will be loaded).

  :param device: Str - The torch device to use (cpu/cuda:N/etc.) GPU is highly recommended.

  :param partially_observable: Bool - whether or not LIDAR scans (and thus observations of the ego-agent) can be blocked
    by objects (walls/other agents/humans etc.)  If False then everything will be observable.

  :param train: Bool - Run training

  :param eval: Bool - Run evaluation

  :param monitor: Bool - Show an RVIS window so you can watch the sim.

  :param local: Bool - If you have graphnav and UTMRS running locally, this will tap the config_runner into those
    ROSModules.  Has some special requirements and isn't recommended.

  :param existence_penalty: Float - Existence Penalty amount (1 will be a reward of -1 per step).  Disabled if 0.

  :param success_reward: Float - Reward amount for reaching the goal. Disabled if 0.

  :param collision_penalty: Float - Penalty amount for colliding with a wall or agent (1 will be a reward of -1 per step).  Disabled if 0.

  :param starting_collision_penalty: Float - Starting weight of the collision penalty if you want to scale it as training continues.

  :param collision_penalty_scale_duration: Float - Number of steps to linearly scale the collision penalty from the
    starting_collision_penalty to the ending collision_penalty value over the specified number of STEPs. Disabled when 0

  :param enforced_order_reward: Float - Penalty and Reward amount for entering/exiting the collision zone under specific
    conditions (controlled via other flags).

  :param enforced_order_track_exit: Bool - Reward/Penalty for agents when they exit the conflict zone

  :param enforced_order_penalty_for_incorrect_order: Bool - Penalty for out of order agents

  :param enforced_order_weak_signal_out_of_zone: Bool - scaled (0.1 * enforced_order_reward) for agents at every
    timestep regardless of if they are in the conflict zone or not (scales to 1 once agents are in a collision zone).

  :param penalty_for_multiple_agents_entering: Bool - Experimental flag that not only gives a penalty for agents out of
    order but also a penalty for when agents try to enter the conflict zone at the same time (agents are overlapping
    with the same edge of a conflict zone at the same time)

  :param goal_distance_reward: Bool - include the goal distance reward.

  :param goal_distance_reward_clip: Bool - clip the goal distance reward to [0, 1]

  :param collision_ender: Bool - End episodes on collisions.

  :param reward_stripper: Bool - When an agent succeeds and remains at the goal state, only give the at goal reward once
    and then 0 from then on (an attempt to make truncation work, truncation is not supported until https://github.com/DLR-RM/stable-baselines3/pull/780
    is resolved.

  :param timelimit: Bool - end episodes on a step timelimit.

  :param timelimit_threshold: Int - the number of steps an episode is allowed to have (if the timelimit bool is set to True)

  :param entropy_ender: Bool - End the episode if agents have not moved a sufficient amount over a length of time.

  :param entropy_max_distance: Float - Distance threshold, if below during the duration of the timelimit the episode
    will end.  1 = one square in RVis (roughly 2meters)

  :param entropy_max_timesteps: Int - number of timesteps that a sufficient distance must be traversed before ending the
    episode

  :param entropy_reward: Bool - If an episode ends during the entropy wrapper, give a reward/penalty to agents determined
    by their final state and configuration flags.

  :param entropy_constant_penalty: Int - If EntropyEnder ends an episode all agents recieve this penalty (1 is equal to -1 reward at the end of the episode).

  :param entropy_constant_penalty_only_those_that_did_not_finish: Bool - Constant penalty only applies if the agents didn't finish (reach their goal)

  :param entropy_reward_multiplier: Int - instead of a constant reward, multiply the current reward (step before the episode ended)

  :param entropy_multiply_negative_rewards_only: Bool - Only multiply rewards that are negative (indicating an agent was "stuck forever")

  :param collision_obs: Bool - Include the collision observation (must be true for the collision penalty)

  :param agent_goal_distance_obs: Bool - Include the distance to the goal as an observation (must be true for the goal distance reward)

  :param agent_pose_obs: Bool - Include the agents pose as an observation (x, y, and theta)

  :param agent_pose_ignore_theta: Bool - Ignore the theta value in the pose observation (orientation)

  :param agent_velocity_obs: Bool - Include the agents velocity as an observation (vx, vy, vtheta)

  :param agent_velocity_ignore_theta: Bool - Ignore the theta value in the velocity observation

  :param other_poses_obs: Bool - Include other agents/humans pose information (x, y, theta) as an observation

  :param other_poses_ignore_theta: Bool - Ignore other agents/humans pose thetas

  :param other_velocities_obs: Include other agents/humans velocities in the observation (vx, vy, vtheta)

  :param other_velocities_ignore_theta: Bool - Ignore the velocity thetas of other agents

  :param policy_algo_sb3_contrib: Bool - True if the policy you want to train has it's base code in the SB3-Contrib library
    i.e. RecurrentPPO

  :param policy_algo_name: Str - Name of the policy class you want to train as it appears in SB3 or SB3-Contrib

  :param policy_name: Str - SB3 Policy structure (MlpPolicy for example)

  :param policy_algo_kwargs: Dict - Any kwargs for the SB3 Policy.

  :param debug: Bool - print statments and ALWAYS GO baseline (gives the action 0 to UTMRS for all agents all the time)

  :param experiment_names: List[str] - List of scenarios currently supported by ConfigRunner (envs_door, envs_hallway,
    envs_intersection, envs_round_about, envs_open)

  :param run_name: Str - Name of the run that will also be used to store tensorbaord info and checkpoints of the model
    as well as evaluation data in `{PROJECT ROOT}/data/{run_name}` (can be a nested folder)

  :param continue_from: Str - Policy checkpoint to load from (relative path from `{PROJECT ROOT}/data`)

  :param run_type: Str - Helper for using SACADRL or AO/EO variants. (allowed values are SACADRL, AO, EO)
  """


  if eval_num_agents is None:
    eval_num_agents = [num_agents] if isinstance(num_agents, int) else [x[1] for x in num_agents]

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
    policy_algo_kwargs = {'verbose': 3, 'device': device, 'n_steps': 512 * (num_agents if isinstance(num_agents, int) else num_agents[-1][1]), 'tensorboard_log': str(exp_tensorboard_folder)}
  if 'device' not in policy_algo_kwargs:
    policy_algo_kwargs['device'] = device
  if 'tensorboard_log' not in policy_algo_kwargs:
    policy_algo_kwargs['tensorboard_log'] = exp_tensorboard_folder
  if 'verbose' not in policy_algo_kwargs:
    policy_algo_kwargs['verbose'] = 3

  conflict_zone = None
  scenarios = []
  zones = []
  if 'envs_door' in experiment_names:
    scenario, conflict_zone = envs_door(partially_observable=partially_observable, config_runner=True if not monitor and not local else False, all_config=monitor and not local)
    scenarios.append(scenario)
    zones.append(conflict_zone)
  if 'envs_hallway' in experiment_names:
    scenario, conflict_zone = envs_hallway(partially_observable=partially_observable, config_runner=True if not monitor and not local else False, all_config=monitor and not local)
    scenarios.append(scenario)
    zones.append(conflict_zone)
  if 'envs_intersection' in experiment_names:
    scenario, conflict_zone = envs_intersection(partially_observable=partially_observable,
                                           config_runner=True if not monitor and not local else False,
                                           all_config=monitor and not local)
    scenarios.append(scenario)
    zones.append(conflict_zone)
  if 'envs_round_about' in experiment_names:
    scenario, conflict_zone = envs_round_about(partially_observable=partially_observable,
                                                config_runner=True if not monitor and not local else False,
                                                all_config=monitor and not local)
    scenarios.append(scenario)
    zones.append(conflict_zone)
  if 'envs_open' in experiment_names:
    scenario, conflict_zone = envs_open(partially_observable=partially_observable,
                                               config_runner=True if not monitor and not local else False,
                                               all_config=monitor and not local)
    scenarios.append(scenario)
    zones.append(conflict_zone)

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
      AgentInZone(),
      EnteringZone(),
      ExitingZone(),
      NumberOfAgentsExitingZone(),
      NumberOfAgentsEnteringZone()
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
      weak_out_of_zone=enforced_order_weak_signal_out_of_zone,
      continuous=enforced_order_weak_signal_out_of_zone,
      penalty_for_multiple_agents_entering=penalty_for_multiple_agents_entering
    ))

  observer = Observer(observations)
  rewarder = Rewarder(rewards)

  # ENV_CLASS = partial(ManualZoneEnv, 7, 11, 1.5)
  if conflict_zone:
    ENV_CLASS = partial(ManualZoneEnv, zones)
  else:
    ENV_CLASS = partial(RosSocialEnv)
  # "eval_num_agents": [2, 3, 4, 5, 7, 10],
  # nav_map_vis = NavMapViz(scenario.nav_map, scenario.nav_lines)
  env = ENV_CLASS(observer=observer, rewarder=rewarder, scenarios=scenarios, num_humans=0, num_agents=num_agents if isinstance(num_agents, int) else max(num_agents[-1][1], eval_num_agents[-1]), debug=debug)

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

  # env = ProgressBarWrapper(env, train_length * (num_agents if isinstance(num_agents, int) else num_agents[-1][1]))


  env = NewScenarioWrapper(env, new_scenario_episode_frequency=1, plans=num_agents if isinstance(num_agents, list) else [0, num_agents])

  env = PerformanceMetricWrapper(
    env,
    tbx_log=f'{run_name}/tensorboard__{run_type}_perf',
  )

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

  #TODO - allow this to work for 1 and 2 agents.
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
    number_of_agents=eval_num_agents,
    tbx_writer=get_tboard_writer(f'{run_name}/tensorboard__{run_type}')
  )

  if train:
    model.learn(
      total_timesteps=train_length * (num_agents if isinstance(num_agents, int) else num_agents[-1][1]),
      callback=eval_callback if eval_frequency > 0 else None,
      tb_log_name=str(exp_tensorboard_folder),
      progress_bar=False
    )

    model.save(exp_checkpoint_folder / 'last')

    if eval_frequency > 0 and ending_eval_with_best:
      model = model.load(exp_checkpoint_folder / 'best_model', env=env, device=device)
    else:
      model = model.load(exp_checkpoint_folder / 'last', env=env, device=device)

  if eval:
    eval_report = {}

    all_rates = []

    for a in tqdm(eval_num_agents, desc='Final Evaluatation....', total=len(eval_num_agents)):
      env.unwrapped.vec_envs[0].par_env.unwrapped.in_eval = True
      env.unwrapped.vec_envs[0].par_env.unwrapped.curr_num_agents = a
      env.unwrapped.vec_envs[0].par_env.unwrapped.new_scenario(num_agents=a)
      env.unwrapped.vec_envs[0].par_env.unwrapped.reset()

      episode_rewards, episode_lengths, success_rate, eval_metrics = evaluate_policy(
        model,
        env,
        n_eval_episodes=ending_eval_trials,
        deterministic=True,
        return_episode_rewards=True,
      )

      eval_report[str(a)] = eval_metrics
      all_rates.append(eval_metrics.get('success_rate'))
    eval_report['total'] = sum(all_rates) / len(all_rates)

    print(eval_report)

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

  import os
  # print(os.system("perf report -g -i perf.data"))
