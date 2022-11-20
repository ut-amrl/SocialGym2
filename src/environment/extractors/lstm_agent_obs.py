import gym
import numpy as np
import torch as th
import torch.nn
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from src.environment.observations import Observer, OtherAgentObservables


class LSTMAgentObs(BaseFeaturesExtractor):
    """
    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(
            self,
            observation_space: gym.spaces.Dict,
            observer: Observer,
            lstm_out_dim: int = 64,
    ):
        self.observer = observer

        self.lstm_input_dim = [x for x in observer.registered_observations[0] if x.name() == OtherAgentObservables.name()][0].agent_obs_length
        self.fusion_input_dim = lstm_out_dim + sum([len(x) for x in observer.registered_observations[0] if x.name() != OtherAgentObservables.name()])


        super(LSTMAgentObs, self).__init__(observation_space, self.fusion_input_dim)


        self.lstm = th.nn.LSTM(self.lstm_input_dim, lstm_out_dim, batch_first=True)

        # self.fusion = nn.Sequential(
        #     nn.Linear(self.fusion_input_dim, fusion_hidden_dim),
        #     nn.ReLU(),
        #     nn.Linear(fusion_hidden_dim, features_dim)
        # )


    def forward(self, observations) -> th.Tensor:
        obs_dicts = self.observer.arr_to_dict(observations)
        lstm_batch, linear_batch = self.make_tensors(obs_dicts)

        lstm_out, _ = self.lstm(lstm_batch)
        lstm_out = lstm_out.squeeze()

        x = torch.concat((linear_batch, lstm_out[:, -1, :]), dim=1)
        # x = self.fusion(x)

        return x

    def make_tensors(self, obs_dicts):

        lstm_batch = []
        linear_batch = []
        for obs_dict in obs_dicts:
            lstm_batch.append(obs_dict.get(OtherAgentObservables.name()).view(-1, self.lstm_input_dim))
            linear_batch.append(th.concat([v for k, v in obs_dict.items() if k != OtherAgentObservables.name()]))

        lstm_batch = th.stack(lstm_batch)
        linear_batch = th.stack(linear_batch)
        return lstm_batch, linear_batch
