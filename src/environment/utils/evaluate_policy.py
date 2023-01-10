import warnings
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import gym
import numpy as np

from stable_baselines3.common import base_class
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv, VecMonitor, is_vecenv_wrapped


def evaluate_policy(
    model: "base_class.BaseAlgorithm",
    env: Union[gym.Env, VecEnv],
    n_eval_episodes: int = 10,
    deterministic: bool = True,
    render: bool = False,
    callback: Optional[Callable[[Dict[str, Any], Dict[str, Any]], None]] = None,
    reward_threshold: Optional[float] = None,
    return_episode_rewards: bool = False,
    warn: bool = True,
) -> Union[Tuple[float, float], Tuple[List[float], List[int]]]:
    """
    SocialGym Note: This is a slightly altered variation of the Stable Baselines V3 evaluate_policy function.
    The major difference is that we check to make sure ALL dones are true (as they refer to the agents being done
    and not an environment being done).  Stable baselines expects an array of dones (similar to how PettingZoo works)
    but, StableBaselines thinks that the array is for a list of environments not a list of agents.

    NEW NOTE: although the above is true, other changes have come about that make this evaluate_policy fn a bit
    different.


    Runs policy for ``n_eval_episodes`` episodes and returns average reward.
    If a vector env is passed in, this divides the episodes to evaluate onto the
    different elements of the vector env. This static division of work is done to
    remove bias. See https://github.com/DLR-RM/stable-baselines3/issues/402 for more
    details and discussion.

    .. note::
        If environment has not been wrapped with ``Monitor`` wrapper, reward and
        episode lengths are counted as it appears with ``env.step`` calls. If
        the environment contains wrappers that modify rewards or episode lengths
        (e.g. reward scaling, early episode reset), these will affect the evaluation
        results as well. You can avoid this by wrapping environment with ``Monitor``
        wrapper before anything else.

    :param model: The RL agent you want to evaluate.
    :param env: The gym environment or ``VecEnv`` environment.
    :param n_eval_episodes: Number of episode to evaluate the agent
    :param deterministic: Whether to use deterministic or stochastic actions
    :param render: Whether to render the environment or not
    :param callback: callback function to do additional checks,
        called after each step. Gets locals() and globals() passed as parameters.
    :param reward_threshold: Minimum expected reward per episode,
        this will raise an error if the performance is not met
    :param return_episode_rewards: If True, a list of rewards and episode lengths
        per episode will be returned instead of the mean.
    :param warn: If True (default), warns user about lack of a Monitor wrapper in the
        evaluation environment.
    :return: Mean reward per episode, std of reward per episode.
        Returns ([float], [int]) when ``return_episode_rewards`` is True, first
        list containing per-episode rewards and second containing per-episode lengths
        (in number of steps).
    """
    is_monitor_wrapped = False
    # Avoid circular import
    from stable_baselines3.common.monitor import Monitor

    if not isinstance(env, VecEnv):
        env = DummyVecEnv([lambda: env])

    is_monitor_wrapped = is_vecenv_wrapped(env, VecMonitor) or env.env_is_wrapped(Monitor)[0]

    if not is_monitor_wrapped and warn:
        warnings.warn(
            "Evaluation environment is not wrapped with a ``Monitor`` wrapper. "
            "This may result in reporting modified episode lengths and rewards, if other wrappers happen to modify these. "
            "Consider wrapping environment first with ``Monitor`` wrapper.",
            UserWarning,
        )

    n_envs = 1  # env.num_envs
    episode_rewards = []
    episode_lengths = []

    episode_counts = np.zeros(n_envs, dtype="int")
    # Divides episodes among different sub environments in the vector as evenly as possible
    episode_count_targets = np.array([(n_eval_episodes + i) // n_envs for i in range(n_envs)], dtype="int")

    current_rewards = np.zeros(n_envs)
    current_lengths = np.zeros(n_envs, dtype="int")
    observations = env.reset()
    states = None
    episode_starts = np.ones((env.num_envs,), dtype=bool)

    successes = 0
    collisions = 0
    total = 0
    lengths = 0
    velocity_delta = 0
    time_still = 0
    incorrect_enter_order = 0
    incorrect_exit_order = 0

    metrics = {}

    prev_collisions = None
    prev_vels = None
    incorrect_enter = None
    incorrect_exit = None

    max_num = 0

    while (episode_counts < episode_count_targets).any():
        actions, states = model.predict(observations, state=states, episode_start=episode_starts, deterministic=deterministic)
        observations, rewards, dones, infos = env.step(actions)
        max_num = sum([1 if 'succeeded' in list(x.keys()) else 0 for x in infos])
        current_rewards += sum(rewards)
        current_lengths += 1
        lengths += 1

        curr_collisions = [x.get('collision', False) for x in infos]
        collisions += sum([curr_collisions[idx] > (prev_collisions[idx] if prev_collisions else 0) for idx in range(max_num)])
        prev_collisions = curr_collisions

        curr_vels = [x.get('velocity', 0) for x in infos]
        velocity_delta += sum([abs(curr_vels[idx] - (prev_vels[idx] if prev_vels else 0)) for idx in range(max_num)])
        prev_vels = curr_vels
        time_still += sum([1 if abs(curr_vels[idx]) < 1e-5 and not dones[idx] else 0 for idx in range(max_num)])

        curr_incorrect_enters = [x.get('incorrect_enter_order', False) for x in infos]
        incorrect_enter_order += sum([curr_incorrect_enters[idx] and not (incorrect_enter[idx] if incorrect_enter else False) for idx in range(max_num)])
        incorrect_enter = [curr_incorrect_enters[idx] or (incorrect_enter[idx] if incorrect_enter else False) for idx in range(max_num)]

        curr_incorrect_exits = [x.get('incorrect_exit_order', False) for x in infos]
        incorrect_exit_order += sum([curr_incorrect_exits[idx] and not (incorrect_exit[idx] if incorrect_exit else False) for idx in range(max_num)])
        incorrect_exit = [curr_incorrect_exits[idx] or (incorrect_exit[idx] if incorrect_exit else False) for idx in range(max_num)]

        if episode_counts[0] < episode_count_targets[0]:

            # unpack values so that the callback can access the local variables
            reward = sum(rewards)
            done = all(dones)
            info = infos
            episode_starts[0] = done

            if callback is not None:
                callback(locals(), globals())

            if all([x for x in dones]):
                if all([x.get('succeeded', False) for x in infos[0:env.unwrapped.vec_envs[0].unwrapped.par_env.unwrapped.curr_num_agents]]):
                    successes += 1
                total += 1

                prev_collisions = None
                prev_vels = None
                incorrect_enter = None
                incorrect_exit = None

                if is_monitor_wrapped:
                    # Atari wrapper can send a "done" signal when
                    # the agent loses a life, but it does not correspond
                    # to the true end of episode
                    if any(["episode" in x.keys() for x in info]):
                        # Do not trust "done" with episode endings.
                        # Monitor wrapper includes "episode" key in info if environment
                        # has been wrapped with it. Use those rewards instead.
                        episode_rewards.append(sum(x["episode"]["r"] for x in infos))
                        episode_lengths.append(sum(x["episode"]["l"] for x in infos))
                        # Only increment at the real end of an episode
                        episode_counts[0] += 1
                else:
                    episode_rewards.append(current_rewards[0])
                    episode_lengths.append(current_lengths[0])
                    episode_counts[0] += 1
                current_rewards[0] = 0
                current_lengths[0] = 0

        if render:
            env.render()

    success_rate = successes / total
    collision_rate = collisions / total
    avg_length = lengths / total
    avg_velocity_delta = velocity_delta / total
    avg_time_still = (time_still / total) / max_num
    incorrect_enter_rate = incorrect_enter_order / total
    incorrect_exit_rate = incorrect_exit_order / total

    metrics['success_rate'] = success_rate
    metrics['collisions'] = collision_rate
    metrics['avg_length'] = avg_length
    metrics['avg_velocity_delta'] = avg_velocity_delta
    metrics['time_still'] = avg_time_still
    metrics['incorrect_enter_rate'] = incorrect_enter_rate
    metrics['incorrect_exit_rate'] = incorrect_exit_rate

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    if reward_threshold is not None:
        assert mean_reward > reward_threshold, "Mean reward below threshold: " f"{mean_reward:.2f} < {reward_threshold:.2f}"
    if return_episode_rewards:
        return episode_rewards, episode_lengths, success_rate, metrics
    return mean_reward, std_reward, success_rate, metrics
