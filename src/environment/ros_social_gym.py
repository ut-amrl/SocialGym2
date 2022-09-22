#!/usr/bin/env python

# Ros Imports
import functools

import roslib

NODE_NAME = 'ros_social_gym'
roslib.load_manifest(NODE_NAME)
from ut_multirobot_sim.srv import utmrsStepper
from ut_multirobot_sim.srv import utmrsReset
from amrl_msgs.srv import SocialPipsSrv
from ut_multirobot_sim.srv import utmrsStepperResponse
import rospy
import roslaunch

# Other Imports
import copy
from copy import deepcopy
import gym
from gym import spaces
from gym.utils import EzPickle, seeding
import json
import numpy as np
import time
import math
from random import seed
from typing import Tuple, Union, TYPE_CHECKING
from pathlib import Path
import shutil
from tensorboardX import SummaryWriter
from pettingzoo import ParallelEnv
from pettingzoo.utils import agent_selector, wrappers

from src.environment.scenarios import Scenario, GraphNavScenario
from src.environment.utils import ROOT_FOLDER

# Package imports
if TYPE_CHECKING:
    from src.environment.rewards import Rewarder
    from src.environment.observations.observer import Observer


class RosSocialEnv(ParallelEnv, EzPickle):
    """
    A ros-based social navigation environment for OpenAI gym supporting MultiAgent training.

    A lot of the documentation is taken from https://pettingzoo.farama.org/content/environment_creation/ which is
    almost entirely about supporting multiple agents in the environment.
    """

    metadata = {
        "render_modes": ["human"],
        "name": "ros_social_env",
    }

    launch_config: str

    observer: 'Observer'
    rewarder: 'Rewarder'

    num_humans: Tuple[int, int]
    num_agents: Tuple[int, int]

    def __init__(
            self,
            launch_config: str = f"{ROOT_FOLDER}/config/gym_gen/launch.launch",
            observer: 'Observer' = None,
            rewarder: 'Rewarder' = None,
            scenario: Scenario = None,
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = (3, 5),
    ):
        """
        ACTIONS: GoAlone, Halt, Follow, Pass

        :param launch_config: Config file that will fire up the Ros Nodes
        :param observer: Observer class object responsible for building env observation vectors per timestep
        :param rewarder: Rewarder class object responsible for determining rewards of the agent per timestep
        :param num_humans: The number of humans to load -- can be a tuple (x, y) where a random number of humans will be
          generated between x and y
        """

        EzPickle.__init__(
            self,
            launch_config,
            deepcopy(observer),
            deepcopy(rewarder),
            deepcopy(scenario),
            num_humans,
            num_agents
        )

        if isinstance(num_agents, int):
            self.ros_num_agents = num_agents, num_agents
        else:
            self.ros_num_agents = num_agents
        if isinstance(num_humans, int):
            self.ros_num_humans = num_humans, num_humans
        else:
            self.ros_num_humans = num_humans

        self.possible_agents = [f"player_{r}" for r in range(max(self.ros_num_agents))]
        self.agent_name_mapping = dict(
            zip(self.possible_agents, list(range(len(self.possible_agents))))
        )

        self.observer = observer
        self.rewarder = rewarder
        self.observer.setup(self)
        self.rewarder.setup(self)

        self.action_spaces = {agent: spaces.Discrete(2) for agent in self.possible_agents}
        self.observation_spaces = {
            agent: spaces.Box(low=-9999, high=9999, shape=(len(self.observer),)) for agent in self.possible_agents
        }

        if scenario is None:
            # TODO - fix; make a better default.
            scenario = GraphNavScenario('closed/door/t1')

        self.scenario = scenario

        self.launch_config = launch_config

        self.num_humans = num_humans

        self.rewarder = rewarder
        self.observer = observer

        self.agents = None
        self.terminations = None

        # Initialize as necessary here
        rospy.init_node('RosSocialEnv', anonymous=True)

        # Launch the simulator launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_config])
        self.launch.start()

        rospy.wait_for_service('utmrsStepper')
        rospy.wait_for_service('utmrsReset')
        self._simStep = rospy.ServiceProxy('utmrsStepper', utmrsStepper)
        self.simReset = rospy.ServiceProxy('utmrsReset', utmrsReset)
        self.pipsSrv = rospy.ServiceProxy('SocialPipsSrv', SocialPipsSrv)

        self.new_scenario()

        self.launch.shutdown()
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_config])
        self.launch.start()

        self.last_obs_maps = []
        self.last_reward_maps = []

    def close(self):
        pass

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]

    def make_observation(self, res):
        return self.observer.make_observation(self, env_response=res)

    def calculate_reward(self, obs_map):
        return self.rewarder.reward(self, obs_map)

    def new_scenario(self):
        self.scenario.generate_scenario(self.ros_num_humans, self.ros_num_agents)

    def default_action(self):
        return [0]

    def sim_step(self, actions):
        return self._simStep(actions)

    def seed(self, seed=None):
        pass

    def reset(self, seed=None, return_info=False, options=None):
        self.agents = self.possible_agents[:]
        self.terminations = {agent: False for agent in self.agents}

        response = self.simReset()
        environment_responses = self.sim_step(self.default_action() * len(self.agents))
        observations, observation_maps = self.make_observation(environment_responses)

        if not return_info:
            return {agent: obs for agent, obs in zip(self.agents, observations)}
        else:
            infos = {
                agent: {} for agent in self.possible_agents if agent in self.agents
            }
            return {agent: obs for agent, obs in zip(self.agents, observations)}, infos

    def step(self, action_dict):
        actions = np.zeros(self.max_num_agents, dtype=np.int32)
        for i, agent in enumerate(self.possible_agents):
            if agent in action_dict:
                actions[i] = action_dict[agent]

        environment_responses = self.sim_step(actions)
        observations, observation_maps = self.make_observation(environment_responses)
        rewards, reward_maps = self.calculate_reward(observation_maps)

        self.last_obs_maps = observation_maps
        self.last_reward_maps = reward_maps

        # TODO - make a "done" observation
        agent_terminations = {agent: False if obs_map['success_observation'] == 0 else True for agent, obs_map in zip(self.agents, observation_maps)}
        agent_observations = {agent: obs for (agent, obs) in zip(self.agents, observations)}
        agent_rewards = {
            agent: reward for agent, reward in zip(self.possible_agents, rewards) if agent in self.agents
        }
        agent_infos = {agent: {} for agent in self.possible_agents if agent in self.agents}
        # self.agents = [agent for agent in self.agents if not agent_terminations[agent]]

        return agent_observations, agent_rewards, agent_terminations, agent_infos

    def render(self, mode="human"):
        """
        Depends on RVIZ for visualization, no render method
        """
        pass
