import pathlib
import pickle

import stable_baselines3 as sb3
from imitation.algorithms import adversarial, bc
from imitation.data import rollout
from imitation.util import logger, util
from torch.utils.data import Dataset, DataLoader
from social_dataset import SocialDataset
from ros_social_gym import RosSocialEnv
from stable_baselines3.common.vec_env import DummyVecEnv

# create Environtment
venv = DummyVecEnv([lambda: RosSocialEnv(1, 1, "config/gym_gen/launch.launch")])

# Train BC on expert data.
# BC also accepts as `expert_data` any PyTorch-style DataLoader that iterates over
# dictionaries containing observations and actions.
transitions = SocialDataset('train_demos.json', 0)
minSamples = len(transitions) * 0.25
maxSamples = len(transitions) - 5
samples = [0, maxSamples * 0.75, maxSamples * 0.5, maxSamples * 0.25, 3500]

for i in samples:
    sample = round(i)
    print(sample)
    transitions = SocialDataset('train_demos.json', sample)
    logger.configure("/home/jaholtz/code/imitation/temp/BC/")
    bc_trainer = bc.BC(venv.observation_space, venv.action_space, expert_data=transitions)
    bc_trainer.train(n_epochs=10)
    bc_trainer.save_policy('bc_policy_' + str(sample))
