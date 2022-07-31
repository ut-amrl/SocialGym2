import stable_baselines3 as sb3

from imitation.algorithms import adversarial
from imitation.util import logger
from social_gail_dataset import SocialDataset
from src.environment.ros_social_gym import RosSocialEnv
from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.policies import serialize
import torch as th
import os
from random import seed

def save(trainer, save_path):
    """Save discriminator and generator."""
    # We implement this here and not in Trainer since we do not want to actually
    # serialize the whole Trainer (including e.g. expert demonstrations).
    os.makedirs(save_path, exist_ok=True)
    th.save(trainer.discrim, os.path.join(save_path, "discrim.pt"))
    # TODO(gleave): unify this with the saving logic in data_collect?
    # (Needs #43 to be merged before attempting.)
    serialize.save_stable_model(
        os.path.join(save_path, "gen_policy"),
        trainer.gen_algo,
        trainer.venv_norm_obs,
    )

# create Environtment
venv = DummyVecEnv([lambda: RosSocialEnv(1, 20, "config/gym_gen/launch.launch")])
seed(1)

transitions = SocialDataset('train_demos.json')
log_dir = "gail_training/"
logger.configure(log_dir)

# Train GAIL on expert data.
# GAIL, and AIRL also accept as `expert_data` any Pytorch-style DataLoader that
# iterates over dictionaries containing observations, actions, and next_observations.
#  logger.configure("/home/jaholtz/code/imitation/temp/BC/")
gail_trainer = adversarial.GAIL(
    venv,
    expert_data=transitions,
    expert_batch_size=100,
    gen_algo=sb3.PPO("MlpPolicy", venv, verbose=1, n_steps=100),
)

checkpoint_interval = 100
print(len(transitions))
print(checkpoint_interval)
maxSamples = len(transitions)
samples = [155400, 116600, 77700, 38800, 3500, 100]

def callback(round_num):
    print(round_num)
    if (round_num * 100) in samples:
        save(gail_trainer, os.path.join(log_dir, "checkpoints", f"{round_num:05d}"))

gail_trainer.train(len(transitions), callback)
