from random import seed
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN, PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from pathlib import Path
import supersuit as ss
from pettingzoo.utils import wrappers

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils import get_tboard_writer, filter_stdout, LogFilter
from src.environment.rewards import Rewarder, Success, GoalDistance, ExistencePenalty, SocialNormPass, SocialNormOvertake, SocialNormCross, Collisions, VelocityControl, PreferredVelocity
from src.environment.rewards.common_rewards import dsacadrl_rewards
from src.environment.observations import Observer, AgentsGoalDistance, AgentsPose, SuccessObservation, \
  AgentsVelocity, AgentsHeadingDirection, AgentsOthersDistance, AgentsPreferredVelocity, OthersPoses, OthersVelocities,\
  OthersHeadingDirection, CollisionObservation
from src.environment.observations.common_observations import dsacadrl_observations
from src.environment.wrappers.new_scenario_wrapper import NewScenarioWrapper
from src.environment.wrappers.time_limit import TimeLimitWrapper
from src.environment.wrappers.simple_multi_agent import SimpleMultiAgent
from src.environment.scenarios import GraphNavScenario
from src.environment.scenarios.common_scenarios import closed_door_1__same_goals, closed_door_1__opposing_sides, elevator_loading
from src.environment.utils import ROOT_FOLDER


seed(1)

tbx_writer, tbx_logdir = get_tboard_writer('dqn_sacadrl')

MODEL_NAME = "tmp"

observations = [
  AgentsGoalDistance(),
  AgentsPose(),
  AgentsVelocity(history_length=2),
  AgentsHeadingDirection(),
  AgentsOthersDistance(),
  AgentsPreferredVelocity(preferred_velocity=1.0),
  OthersPoses(),
  OthersVelocities(),
  OthersHeadingDirection(),
  SuccessObservation(),
  CollisionObservation()
]

rewards = [
  ExistencePenalty(0.1),
  Success(),
  SocialNormCross(weight=33),
  SocialNormOvertake(weight=33),
  SocialNormPass(weight=33),
  Collisions(weight=100),
  VelocityControl(weight=.1),
  PreferredVelocity()
]

observer = Observer(observations)
rewarder = Rewarder(rewards)

EPISODE_LENGTH = 2_000
TRAIN_LENGTH = 5_000

# scenario = GraphNavScenario('elevator/t1')
scenario = closed_door_1__same_goals('t1')
# scenario = elevator_loading()

env = RosSocialEnv(observer=observer, rewarder=rewarder, scenario=scenario, num_humans=0, num_agents=2)
# env = wrappers.AssertOutOfBoundsWrapper(env)
# env = wrappers.OrderEnforcingWrapper(env)
env = ss.pettingzoo_env_to_vec_env_v1(env)
env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')
# env = ss.concat_vec_envs_v1(env, 4, num_cpus=2, base_class="stable_baselines3")
# env = SimpleMultiAgent(env, None, number_of_agents=2)
# env = NewScenarioWrapper(env, new_scenario_episode_frequency=1)
# env = TimeLimitWrapper(env, max_steps=EPISODE_LENGTH)

# env = Monitor(env)
# env = DummyVecEnv([lambda: env])



GYM_TBX = False
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tbx_logdir if GYM_TBX else None)

# TODO - fix this, the multi-agent should probably be an algorithm not an env wrapper (or maybe both)
# env.envs[0].env.env.env.model = model

model.learn(EPISODE_LENGTH * TRAIN_LENGTH, eval_env=env, eval_freq=EPISODE_LENGTH * 5, n_eval_episodes=5)

model_file = Path("baselines")
model_file.mkdir(parents=True, exist_ok=True)

model.save(str(model_file / f'{MODEL_NAME}'))
