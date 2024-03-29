import os
from pathlib import Path
import json
import warnings
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Union

import gym
import numpy as np

from stable_baselines3.common import base_class  # pytype: disable=pyi-error
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv, sync_envs_normalization
from stable_baselines3.common.callbacks import EventCallback, BaseCallback

from src.environment.utils.evaluate_policy import evaluate_policy


class EvalCallback(EventCallback):
    """
    Callback for evaluating an agent.

    .. warning::

      When using multiple environments, each call to  ``env.step()``
      will effectively correspond to ``n_envs`` steps.
      To account for that, you can use ``eval_freq = max(eval_freq // n_envs, 1)``

    :param eval_env: The environment used for initialization
    :param callback_on_new_best: Callback to trigger
        when there is a new best model according to the ``mean_reward``
    :param callback_after_eval: Callback to trigger after every evaluation
    :param n_eval_episodes: The number of episodes to test the agent
    :param eval_freq: Evaluate the agent every ``eval_freq`` call of the callback.
    :param log_path: Path to a folder where the evaluations (``evaluations.npz``)
        will be saved. It will be updated at each evaluation.
    :param best_model_save_path: Path to a folder where the best model
        according to performance on the eval env will be saved.
    :param deterministic: Whether the evaluation should
        use a stochastic or deterministic actions.
    :param render: Whether to render or not the environment during evaluation
    :param verbose:
    :param warn: Passed to ``evaluate_policy`` (warns if ``eval_env`` has not been
        wrapped with a Monitor wrapper)
    """

    def __init__(
        self,
        eval_env: Union[gym.Env, VecEnv],
        callback_on_new_best: Optional[BaseCallback] = None,
        callback_after_eval: Optional[BaseCallback] = None,
        n_eval_episodes: int = 5,
        eval_freq: int = 10000,
        log_path: Optional[str] = None,
        best_model_save_path: Optional[str] = None,
        deterministic: bool = True,
        render: bool = False,
        verbose: int = 1,
        warn: bool = True,
        use_mean_reward: bool = False,
        use_success_rate: bool = True,
        eval_report_file: Path = None,
        number_of_agents: List[int] = None,
        tbx_writer = None
    ):
        super().__init__(callback_after_eval, verbose=verbose)

        self.callback_on_new_best = callback_on_new_best
        if self.callback_on_new_best is not None:
            # Give access to the parent
            self.callback_on_new_best.parent = self

        self.n_eval_episodes = n_eval_episodes
        self.eval_freq = eval_freq
        self.best_mean_reward = -np.inf
        self.last_mean_reward = -np.inf
        self.deterministic = deterministic
        self.render = render
        self.warn = warn
        self.use_mean_reward = use_mean_reward
        self.use_success_rate = use_success_rate
        self.eval_report_file = eval_report_file
        self.number_of_agents = number_of_agents

        self.best_success_rate = 0.
        self.tbx_writer = tbx_writer

        # Convert to VecEnv for consistency
        if not isinstance(eval_env, VecEnv):
            eval_env = DummyVecEnv([lambda: eval_env])

        self.eval_env = eval_env
        self.best_model_save_path = best_model_save_path
        # Logs will be written in ``evaluations.npz``
        if log_path is not None:
            log_path = os.path.join(log_path, "evaluations")
        self.log_path = log_path
        self.evaluations_results = []
        self.evaluations_timesteps = []
        self.evaluations_length = []
        # For computing success rate
        self._is_success_buffer = []
        self.evaluations_successes = []

        self.eval_number = 0

    def _init_callback(self) -> None:
        # Does not work in some corner cases, where the wrapper is not the same
        if not isinstance(self.training_env, type(self.eval_env)):
            warnings.warn("Training and eval env are not of the same type" f"{self.training_env} != {self.eval_env}")

        # Create folders if needed
        if self.best_model_save_path is not None:
            os.makedirs(self.best_model_save_path, exist_ok=True)
        if self.log_path is not None:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

        # Init callback called on new best model
        if self.callback_on_new_best is not None:
            self.callback_on_new_best.init_callback(self.model)

    def _log_success_callback(self, locals_: Dict[str, Any], globals_: Dict[str, Any]) -> None:
        """
        Callback passed to the  ``evaluate_policy`` function
        in order to log the success rate (when applicable),
        for instance when using HER.

        :param locals_:
        :param globals_:
        """
        info = locals_["info"]

        if locals_["done"]:
            maybe_is_success = all([x.get("is_success") for x in info])
            if maybe_is_success is not None:
                self._is_success_buffer.append(maybe_is_success)

    def _on_step(self) -> bool:
        assert len(self.eval_env.unwrapped.vec_envs) == 1, \
            'Evaluation Callback cannot handle more than 1 vectorized env atm.'

        continue_training = True

        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            prev_agent_number = self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.curr_num_agents

            # Sync training and eval env if there is VecNormalize
            if self.model.get_vec_normalize_env() is not None:
                try:
                    sync_envs_normalization(self.training_env, self.eval_env)
                except AttributeError as e:
                    raise AssertionError(
                        "Training and eval env are not wrapped the same way, "
                        "see https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html#evalcallback "
                        "and warning above."
                    ) from e

            # Reset success rate buffer
            self._is_success_buffer = []

            self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.in_eval = True


            _reset_number_of_agents = False
            if self.number_of_agents is None:
                _reset_number_of_agents = True
                self.number_of_agents = [prev_agent_number]

            episode_rewards = np.array([])
            episode_lengths = np.array([])
            success_rate = 0.

            success_rate_per_agents = {}
            all_metrics = {}

            for num_agents in self.number_of_agents:
                self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.curr_num_agents = num_agents
                self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.new_scenario(num_agents=num_agents)
                self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.reset()

                rwds, lens, scssrate, metrics = evaluate_policy(
                    self.model,
                    self.eval_env,
                    n_eval_episodes=self.n_eval_episodes,
                    render=self.render,
                    deterministic=self.deterministic,
                    return_episode_rewards=True,
                    warn=self.warn,
                    callback=self._log_success_callback,
                )

                all_metrics[f'{num_agents}_agents'] = metrics
                if self.tbx_writer:
                    self.tbx_writer.add_scalars(f'eval_{num_agents}_agents', metrics, self.eval_number)

                episode_rewards = np.concatenate([episode_rewards, rwds])
                episode_lengths = np.concatenate([episode_lengths, lens])
                success_rate += scssrate
                success_rate_per_agents[str(num_agents)] = scssrate

            success_rate /= len(self.number_of_agents)

            if _reset_number_of_agents:
                self.number_of_agents = None

            if self.log_path is not None:
                self.evaluations_timesteps.append(self.num_timesteps)
                self.evaluations_results.append(episode_rewards)
                self.evaluations_length.append(episode_lengths)

                kwargs = {}
                # Save success log if present
                if len(self._is_success_buffer) > 0:
                    self.evaluations_successes.append(self._is_success_buffer)
                    kwargs = dict(successes=self.evaluations_successes)

                np.savez(
                    self.log_path,
                    timesteps=self.evaluations_timesteps,
                    results=self.evaluations_results,
                    ep_lengths=self.evaluations_length,
                    **kwargs,
                )

            mean_reward, std_reward = np.mean(episode_rewards), np.std(episode_rewards)
            mean_ep_length, std_ep_length = np.mean(episode_lengths), np.std(episode_lengths)
            self.last_mean_reward = mean_reward
            self.last_success_rate = success_rate
            self.eval_number += 1

            if self.eval_report_file is not None:
                eval_report = {}
                if self.eval_report_file.exists():
                    eval_report = json.load(self.eval_report_file.open('r'))
                eval_report[str(self.n_calls)] = {'total': success_rate, **success_rate_per_agents, **all_metrics}
                json.dump(eval_report, self.eval_report_file.open('w'))

            if self.verbose > 0:
                print(f"Eval num_timesteps={self.num_timesteps}, " f"episode_reward={mean_reward:.2f} +/- {std_reward:.2f}")
                print(f"Episode length: {mean_ep_length:.2f} +/- {std_ep_length:.2f}")
            # Add to current Logger
            self.logger.record("eval/mean_reward", float(mean_reward))
            self.logger.record("eval/mean_ep_length", mean_ep_length)

            if len(self._is_success_buffer) > 0:
                success_rate = np.mean(self._is_success_buffer)
                if self.verbose > 0:
                    print(f"Success rate: {100 * success_rate:.2f}%")
                self.logger.record("eval/success_rate", success_rate)

            # Dump log so the evaluation results are printed with the correct timestep
            self.logger.record("time/total_timesteps", self.num_timesteps, exclude="tensorboard")
            self.logger.dump(self.num_timesteps)

            # if (self.use_mean_reward and mean_reward > self.best_mean_reward) or (self.use_success_rate and success_rate > self.best_success_rate):
            if success_rate >= self.best_success_rate:
                if self.verbose > 0:
                    print("New best mean reward!")
                if self.best_model_save_path is not None:
                    print("SAVING")
                    self.model.save(os.path.join(self.best_model_save_path, "best_model"))

                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                # if success_rate > self.best_success_rate:
                self.best_success_rate = success_rate

                # Trigger callback on new best model, if needed
                if self.callback_on_new_best is not None:
                    continue_training = self.callback_on_new_best.on_step()

            # Trigger callback after every evaluation, if needed
            if self.callback is not None:
                continue_training = continue_training and self._on_event()

            # print(f'EVAL FIN: {prev_agent_number}')
            self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.curr_num_agents = prev_agent_number
            self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.new_scenario(num_agents=prev_agent_number)
            self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.reset()

        self.eval_env.unwrapped.vec_envs[0].par_env.unwrapped.in_eval = False

        return continue_training

    def update_child_locals(self, locals_: Dict[str, Any]) -> None:
        """
        Update the references to the local variables.

        :param locals_: the local variables during rollout collection
        """
        if self.callback:
            self.callback.update_locals(locals_)


class OptunaCallback(BaseCallback):
    """
    Stop the training once a threshold in episodic reward
    has been reached (i.e. when the model is good enough).

    It must be used with the ``EvalCallback``.

    :param reward_threshold:  Minimum expected reward per episode
        to stop training.
    :param verbose:
    """

    def __init__(self, trial, verbose: int = 0):
        super().__init__(verbose=verbose)
        self.trial = trial
        self.eval_num = 0

    def _on_step(self) -> bool:
        assert self.parent is not None, "``StopTrainingOnMinimumReward`` callback must be used " "with an ``EvalCallback``"
        eval_number = self.parent.eval_number

        if self.eval_num != eval_number:
            if self.parent.use_mean_reward:
                s = self.parent.last_mean_reward
            else:
                s = self.parent.last_success_rate

            self.trial.report(s, self.eval_num)
            self.eval_num = eval_number
            self.trial.set_user_attr("score", s)

        continue_training = not self.trial.should_prune()

        return continue_training
