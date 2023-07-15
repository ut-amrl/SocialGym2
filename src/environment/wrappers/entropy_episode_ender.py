import gym
import numpy as np
import collections
from typing import Union, Tuple, Dict, List
from pettingzoo.utils.wrappers import BaseParallelWrapper

GymObs = Union[Tuple, Dict, np.ndarray, int]


class EntropyEpisodeEnder(BaseParallelWrapper):
    """
    You stop moving you die! (if all agents in an environment are no longer moving within some absolute delta over a
    set of timesteps, end the episode)

    :param env: Gym environment that will be wrapped
    """

    distance_delta: float
    timestep_threshold: int
    reward_multiplier: float
    negative_multiplier_only: bool
    agent_positions: List[List[List[float]]]

    def __init__(
            self,
            env: gym.Env,
            distance_delta: float = 0.01,
            timestep_threshold: int = 100,
            reward_multiplier: float = 100_000,
            negative_multiplier_only: bool = False,
            constant_reward_on_end: float = None,
            only_those_that_did_not_finish: bool = False,
            update_rewards: bool = True
    ):
        # Call the parent constructor, so we can access self.env later
        super().__init__(env)

        self.distance_delta = distance_delta
        self.timestep_threshold = timestep_threshold
        self.reward_multiplier = reward_multiplier
        self.negative_multiplier_only = negative_multiplier_only
        self.constant_reward_on_end = constant_reward_on_end
        self.only_those_that_did_not_finish = only_those_that_did_not_finish
        self.update_rewards = update_rewards

        # Every agent gets a circular queue of positions that we will use to calculate the total delta of movement
        self.agent_positions = []

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        obs, reward, done, infos = self.env.step(action)
        self.agents = self.unwrapped.agents

        [self.agent_positions[idx].append(m['agents_pose']) for idx, m in enumerate(self.unwrapped.last_obs_maps)]
        deltas = self.calculate_deltas()

        stuck = all([-1 < x < self.distance_delta for x in deltas])

        if stuck:
            done = {k: True for k in done.keys()}

            if self.update_rewards:
                if self.constant_reward_on_end:
                    reward = {k: self.constant_reward_on_end if ((self.only_those_that_did_not_finish and not infos.get(k, {}).get('succeeded', False)) or not self.only_those_that_did_not_finish) else v for idx, (k, v)  in enumerate(reward.items())}
                    print(reward)
                elif self.reward_multiplier:
                    # Only multiple negative rewards? Maybe a hack?
                    if self.negative_multiplier_only:
                        reward = {k: v * self.reward_multiplier if v < 0 and ((self.only_those_that_did_not_finish and not infos.get(k, {}).get('succeeded', False)) or not self.only_those_that_did_not_finish) else v for idx, (k, v) in enumerate(reward.items())}
                    else:
                        reward = {k: v * self.reward_multiplier if ((self.only_those_that_did_not_finish and not infos.get(k, {}).get('succeeded', False)) or not self.only_those_that_did_not_finish) else v for idx, (k, v) in enumerate(reward.items())}

        return obs, reward, done, infos

    def reset(self, seed=None, return_info=False, options=None):
        res = self.env.reset(seed=seed, options=options)
        self.reset_positions()
        self.agents = self.unwrapped.agents
        return res

    def reset_positions(self):
        self.agent_positions = [collections.deque(maxlen=self.timestep_threshold) for _ in range(self.unwrapped.curr_num_agents)]

    def calculate_deltas(self):
        deltas = [-1] * self.unwrapped.curr_num_agents
        for idx, positions in enumerate(self.agent_positions):
            if idx >= len(deltas):
                break
            if len(positions) < self.timestep_threshold:
                continue

            delta = 0
            for i in range(0, len(positions)-1):
                cur = positions[i]
                next = positions[i+1]
                delta += np.linalg.norm(cur[0:2] - next[0:2], ord=2)
            deltas[idx] = delta
        return deltas

    def seed(self, seed=None):
        pass
