from typing import List, Dict, TYPE_CHECKING

import numpy as np

from src.environment.rewards import Reward

if TYPE_CHECKING:
    from src.environment.ros_social_gym import RosSocialEnv


class Rewarder:

    registered_rewards: List[Reward]

    def __init__(self, registered_rewards: List[Reward]):
        self.registered_rewards = registered_rewards

    def reward(self, env: 'RosSocialEnv', env_response, data_map) -> np.array:
        observation_map = env.observer.make_observation_map(env, env_response)
        return sum([x(env, env_response, observation_map, data_map) for x in self.registered_rewards])

    def reward_map(self, env: 'RosSocialEnv', env_response, data_map) -> Dict[str, float]:
        observation_map = env.observer.make_observation_map(env, env_response)
        return {reward.name(): reward(env, env_response, observation_map, data_map) for reward in self.registered_rewards}

    def setup(self, env: 'RosSocialEnv'):
        [x.__setup__(env) for x in self.registered_rewards]
