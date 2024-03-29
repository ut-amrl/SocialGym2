import optuna
import torch as th
import json
from random import seed
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList
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


exp_folder = DATA_FOLDER / '11_7_22_exps'
exp_report = exp_folder / 'report.json'
exp_checkpoint_folder = exp_folder / 'checkpoints'
exp_tensorboard_folder = exp_folder / 'tensorboard'

report = {}
trial_num = '5a_eo_major_reward__thetas__low_col'

if exp_report.exists():
  with exp_report.open('r') as f:
    report = json.load(f)


def objective(trial):
  global exp_folder, exp_report, exp_checkpoint_folder, exp_tensorboard_folder, report, trial_num

  NUMBER_OF_AGENTS = 5
  TRAIN_LENGTH = 100_000
  INTERMEDIATE_EVAL_TRIALS = 10
  ENDING_EVAL_TRIALS = 100
  scenario = exp1_train_scenario(level='easy', partially_observable=False)

  relative_other_poses = False #trial.suggest_categorical("relative_other_poses", [True, False])

  existence_penalty = 1.0 #trial.suggest_float("existence_penalty", 0.5, 1.0, step=0.1)
  success_reward = 100  # trial.suggest_categorical("success_reward", [1, 10, 100])

  collisions_max_penalty = 20 #trial.suggest_int("collision_max_penalty", 30, 100, step=10)
  collision_min_penalty = 0.5 #trial.suggest_float("collision_min_penalty", 0.3, 1.0, step=0.05)
  collision_scale_duration = 40 #trial.suggest_int("collision_scale_duration", 20, 50, step=10)

  enforced_order_reward = 50 #trial.suggest_int("enforced_order_reward", 10, 100, step=10)
  enforced_order_reward_on_exit = True  # trial.suggest_categorical("enforced_order_reward_on_exit", [True, False])
  enforced_order_reward_incorrect_penalty = True  # trial.suggest_categorical("enforced_order_reward_incorrect_penalty", [True, False])

  entropy_max_distance = 0.5 #trial.suggest_float("entropy_max_distance", 0.3, 1, step=0.1)
  entropy_max_timesteps = 50 #trial.suggest_int("entropy_max_timesteps", 60, 120, step=10)
  # entropy_constant_penalty = trial.suggest_categorical("entropy_constant_penalty", [True, False])
  # entropy_multiply_negative_rewards_only = trial.suggest_categorical("entropy_multiply_negative_rewards", [True, False])
  entropy_only_penalize_agents_that_did_not_finish = True

  entropy_max_timesteps *= NUMBER_OF_AGENTS

  observations = [
    AgentsGoalDistance(history_length=2),
    AgentsPose(ignore_theta=False),
    AgentsVelocity(history_length=2, ignore_theta=False),
    OthersPoses(actuals=relative_other_poses, ignore_theta=False),
    OthersVelocities(ignore_theta=False),
    SuccessObservation(),
    CollisionObservation(),
    AgentZonePriorityOrder(),
    AgentZoneCurrentOrder(),
    AgentInZone(),
  ]

  rewards = [
    ExistencePenalty(existence_penalty),
    Success(weight=success_reward),
    LinearWeightScheduler(Collisions(weight=1.0), min_weight=collision_min_penalty, max_weight=collisions_max_penalty, duration=collision_scale_duration),
    GoalDistanceChange(),
    EnforcedOrder(weight=enforced_order_reward, on_enter=True, on_exit=enforced_order_reward_on_exit, continuous=False, weak_out_of_zone=False,
                  allow_any_order=False, incorrect_penalty=enforced_order_reward_incorrect_penalty)
  ]

  observer = Observer(observations)
  rewarder = Rewarder(rewards)

  MODEL_NAME = f'{exp_folder.name}/{exp_tensorboard_folder.name}/{trial_num}'
  TBX_LOG = MODEL_NAME
  CHECKPOINT_PATH = exp_checkpoint_folder / MODEL_NAME

  CHECKPOINT_PATH.mkdir(parents=True, exist_ok=True)

  # ENV_CLASS = RosSocialEnv
  ENV_CLASS = partial(ManualZoneEnv, 7, 11, 1.5)

  env = ENV_CLASS(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=NUMBER_OF_AGENTS)
  env = EntropyEpisodeEnder(env, timestep_threshold=entropy_max_timesteps, distance_delta=entropy_max_distance, negative_multiplier_only=False, constant_reward_on_end=-100_000, only_those_that_did_not_finish=entropy_only_penalize_agents_that_did_not_finish)
  env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)

  env = TensorboardWriter(env, tbx_log=TBX_LOG, record_video=False, record_rewards=True, video_sample_rate=1,
                            step_sample_rate=1)

  env = ss.pettingzoo_env_to_vec_env_v1(env)

  env = ss.concat_vec_envs_v1(env, 1, num_cpus=10, base_class='stable_baselines3')

  # policy_kwargs = dict(activation_fn=th.nn.ReLU,
  #                      net_arch=[dict(pi=[128, 128], vf=[128, 128])])

  # model = PPO("MlpPolicy", env, verbose=3, device='cuda:1', policy_kwargs=policy_kwargs)
  model = PPO("MlpPolicy", env, verbose=3, device='cuda:1', n_steps=2048 * NUMBER_OF_AGENTS)

  # optuna_callback = OptunaCallback(
  #   trial=trial
  # )

  eval_callback = EvalCallback(
    eval_env=env,
    n_eval_episodes=INTERMEDIATE_EVAL_TRIALS,
    eval_freq=10_000,
    best_model_save_path=CHECKPOINT_PATH,
    # callback_after_eval=optuna_callback
  )

  with (CHECKPOINT_PATH / 'config.json').open('w') as f:
    json.dump({
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

  model.learn(TRAIN_LENGTH * NUMBER_OF_AGENTS, callback=eval_callback)
  model.save(CHECKPOINT_PATH / 'last')

  if (CHECKPOINT_PATH / 'best').exists():
    model = model.load(CHECKPOINT_PATH / 'best', env=env, device='cuda:1')
  else:
    model = model.load(CHECKPOINT_PATH / 'last', env=env, device='cuda:1')

  episode_rewards, episode_lengths, success_rate = evaluate_policy(
    model,
    model.get_env(),
    n_eval_episodes=ENDING_EVAL_TRIALS,
    deterministic=True,
    return_episode_rewards=True,
  )

  report[str(trial_num)] = success_rate
  with exp_report.open('w') as f:
    json.dump(report, f)

  #trial_num += 1


if __name__ == "__main__":
  from optuna.trial import TrialState

  # study = optuna.create_study(direction="maximize")
  # study.optimize(objective, n_trials=100, gc_after_trial = True)
  #
  # pruned_trials = study.get_trials(deepcopy=False, states=[TrialState.PRUNED])
  # complete_trials = study.get_trials(deepcopy=False, states=[TrialState.COMPLETE])
  #
  # print("Study statistics: ")
  # print("  Number of finished trials: ", len(study.trials))
  # print("  Number of pruned trials: ", len(pruned_trials))
  # print("  Number of complete trials: ", len(complete_trials))
  #
  # print("Best trial:")
  # trial = study.best_trial
  #
  # print("  Value: ", trial.value)
  #
  # print("  Params: ")
  # for key, value in trial.params.items():
  #     print("    {}: {}".format(key, value))
  objective(None)