import os
import json
from torch.utils.data import Dataset, DataLoader
import random
import copy

class SocialDataset(Dataset):
    def __init__(self, data_path, dataSize):
        with open(data_path) as json_file:
            self.data_list = json.load(json_file)
            self.data_list.pop(0)
            if (dataSize != 0):
                self.data_list = self.data_list[dataSize:]

    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        temp = copy.deepcopy(self.data_list[idx])
        temp['obs'] = [temp['obs']]
        return temp
