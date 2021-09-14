import os
import json
import numpy as np
import copy
from torch.utils.data import Dataset, DataLoader

batch_size = 100
class SocialDataset(Dataset):
    def __init__(self, data_path):
        with open(data_path) as json_file:
            self.data_list = json.load(json_file)
            self.data_list.pop(0)
            self.last_data = np.array([])

    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        if (idx + batch_size < len(self.data_list)):
            item = copy.deepcopy(self.data_list[idx])
            item['obs'] = np.array([item['obs']])
            item['next_obs'] = np.array([item['next_obs']])
            item['acts'] =  np.array([item['acts']])
            item['dones'] =  np.array([False])
            for i in range(idx + 1, idx + batch_size):
                temp = self.data_list[idx + i]
                item['obs'] = np.concatenate((item['obs'], [temp['obs']]), axis=0)
                item['next_obs'] = np.concatenate((item['next_obs'], [temp['next_obs']]), axis=0)
                item['acts'] = np.concatenate((item['acts'], [temp['acts']]))
                item['dones'] = np.concatenate((item['dones'], [False]))

            self.last_data = item
            return item
        return self.last_data
