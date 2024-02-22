#!/usr/bin/env python

# Ros Imports
import functools

import roslib

NODE_NAME = 'ros_social_gym'
roslib.load_manifest(NODE_NAME)
from amrl_msgs.srv import SocialPipsSrv
import rospy
import roslaunch

# Other Imports
import copy
import gymnasium as gym
from copy import deepcopy
# import gym
from gymnasium import spaces
# from gym import spaces
from gym.utils import EzPickle, seeding
import json
import numpy as np
import time
import math
from random import seed
from typing import Tuple, Union, ClassVar, List, TYPE_CHECKING
from pathlib import Path
import shutil
import pprint
from tensorboardX import SummaryWriter
from pettingzoo import ParallelEnv
from pettingzoo.utils import agent_selector, wrappers

from src.environment.services import UTMRS, UTMRSResponse
from src.environment.scenarios import Scenario, GraphNavScenario
from src.environment.utils.utils import ROOT_FOLDER
from src.environment.visuals.nav_map_viz import NavMapViz

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

    def state(self) -> np.ndarray:
        pass

    metadata = {
        "render_modes": ["human"],
        "name": "ros_social_env",
    }

    launch_config: str

    observer: 'Observer'
    rewarder: 'Rewarder'

    num_humans: Tuple[int, int]
    num_agents: Tuple[int, int]

    env_response_type: ClassVar[UTMRSResponse]

    def __new__(
            cls,
            *args,
            launch_config: str = f"{ROOT_FOLDER}/config/gym_gen/launch.launch",
            observer: 'Observer' = None,
            rewarder: 'Rewarder' = None,
            scenarios: List[Scenario] = (),
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = (3, 5),
            debug: bool = False,
            visuals=None,
            **kwargs
    ):
        """ creates a singleton object, if it is not created,
        or else returns the previous singleton object"""

        instance_name = f'instance'
        if False:
            return super(RosSocialEnv, cls).__new__(cls)

        if not hasattr(cls, instance_name):
            setattr(cls, instance_name, super(RosSocialEnv, cls).__new__(cls))
        return getattr(cls, instance_name)

    def __init__(
            self,
            *args,
            launch_config: str = f"{ROOT_FOLDER}/config/gym_gen/launch.launch",
            observer: 'Observer' = None,
            rewarder: 'Rewarder' = None,
            scenarios: List[Scenario] = (),
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = (3, 5),
            debug: bool = False,
            visuals=None,
            **kwargs
    ):
        """
        ACTIONS: GoAlone, Halt, Follow, Pass

        :param launch_config: Config file that will fire up the Ros Nodes
        :param observer: Observer class object responsible for building env observation vectors per timestep
        :param rewarder: Rewarder class object responsible for determining rewards of the agent per timestep
        :param num_humans: The number of humans to load -- can be a tuple (x, y) where a random number of humans will be
          generated between x and y
        """

        if hasattr(self, 'instantiated'):
            return
        
        EzPickle.__init__(
            self,
            *args,
            launch_config,
            deepcopy(observer),
            deepcopy(rewarder),
            deepcopy(scenarios),
            num_humans,
            num_agents,
            debug,
            **kwargs
        )


        self.black_death = True

        self.debug = debug
        self.in_eval = False
        self.visuals = visuals
        self.scenarios = scenarios
        self.scenario_idx = 0

        if isinstance(num_agents, int):
            self.ros_num_agents = num_agents, num_agents
        else:
            self.ros_num_agents = num_agents
        if isinstance(num_humans, int):
            self.ros_num_humans = num_humans, num_humans
        else:
            self.ros_num_humans = num_humans

        self.curr_num_agents = max(self.ros_num_agents)


        self.possible_agents = [f"player_{r}" for r in range(max(self.ros_num_agents))]
        self.real_possible_agents = [f"player_{r}" for r in range(max(self.ros_num_agents))]

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

        self.new_scenario()


        self.launch_config = launch_config

        self.num_humans = num_humans

        self.rewarder = rewarder
        self.observer = observer

        self.agents = None
        self.terminations = None


        self.last_obs_maps = []
        self.last_reward_maps = []
        self.terminations_ = [False] * len(self.possible_agents)



        # Initialize as necessary here
        rospy.init_node('RosSocialEnv', anonymous=True)

        # Launch the simulator launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_config])
        self.launch.start()

        # time.sleep(5)

        self.utmrs_service = UTMRS()
        self.env_response_type = UTMRSResponse

        self.pipsSrv = rospy.ServiceProxy('SocialPipsSrv', SocialPipsSrv)


        # self.launch.shutdown()

        # time.sleep(5)

        # self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_config])
        # self.launch.start()

        self.instantiated = True
   

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

    def new_scenario(self, num_humans=None, num_agents=None):
        # print("NEW SCENARIO", flush=True)
        # if num_agents:
        #     self.ros_num_agents = [num_agents, num_agents]
        #     print(f"UPDATING AGENTS: {num_agents}")
        #     self.possible_agents = [f"player_{r}" for r in range(max(self.ros_num_agents))]
        #     print(f"UPDATING LENGTH OF POSSIBLE AGENTS: {len(self.possible_agents)}", flush=True)
        #     self.real_possible_agents = [f"player_{r}" for r in range(max(self.ros_num_agents))]
        
        #     self.agent_name_mapping = dict(
        #         zip(self.possible_agents, list(range(len(self.possible_agents))))
        #     )
        
        #     self.action_spaces = {agent: spaces.Discrete(2) for agent in self.possible_agents}
        #     self.observer.setup(self)
        #     self.rewarder.setup(self)
        #     self.observation_spaces = {
        #         agent: spaces.Box(low=-9999, high=9999, shape=(len(self.observer),)) for agent in self.possible_agents
        #     }


        self.scenario_idx = (self.scenario_idx + 1) % len(self.scenarios)
        self.scenarios[self.scenario_idx].generate_scenario(num_humans if num_humans else self.ros_num_humans, num_agents if num_agents else self.curr_num_agents)
        # Loop through scenarios
        return None

    def default_action(self):
        actions = [[0] * self.curr_num_agents, [0.] * self.curr_num_agents,  [0.] * self.curr_num_agents, [0.] * self.curr_num_agents, [-1.] * self.curr_num_agents, [f'{i}' for i in range(self.curr_num_agents)], [AgentColor() for i in range(self.curr_num_agents)]]
        # actions = [[0] * self.curr_num_agents, [0.] * self.curr_num_agents,  [0.] * self.curr_num_agents, [0.] * self.curr_num_agents, [f'{i}' for i in range(self.curr_num_agents)], [AgentColor() for i in range(self.curr_num_agents)]]
        return actions

    def sim_step(self, args):
        return self.env_response_type.process(self.utmrs_service.step(*args))

    def seed(self, seed=None):
        pass

    def reset(self, seed=None, return_info=True, options=None):
        # if self.debug:
        #     print("RESET")
        # print("RESET")
        scenario = self.scenarios[self.scenario_idx]
        self.visuals = [NavMapViz(scenario.nav_map, scenario.nav_lines)]

        pause = self.agents is None or len(self.agents) != self.curr_num_agents

        self.agents = self.real_possible_agents[:]
        self.possible_agents = self.agents
        self.terminations = {agent: True if self.curr_num_agents <= idx else False for idx, agent in enumerate(self.agents)}
        self.terminations_ = [True if self.curr_num_agents <= idx else False for idx, _ in enumerate(self.agents)]

        self.observer.reset()
        self.rewarder.reset()

        # if self.debug:
        #     print(f'Curr Num Agents: {self.curr_num_agents}')
        #     print(f'Length of default action: {len(self.default_action()[0])}')

        NUM_RETRIES = 1_000

        time.sleep(1)

        retry = 0
        environment_responses = None
        while retry < NUM_RETRIES:
            try:
                self.utmrs_service.reset()
                if pause:
                    time.sleep(3)

                environment_responses = self.sim_step(
                    self.default_action()
                )

                if pause:
                    self.utmrs_service.reset()

                # if self.debug:
                #     print(f'Env Resp Length: {len(environment_responses)}')

                if len(environment_responses) == self.curr_num_agents:
                    break

                # if self.debug:
                #     print('fail check, retrying')

                retry += 1
                time.sleep(1)
            except Exception as e:
                # if self.debug:
                #     print('fail check, retrying')
                print(e)
                retry += 1
                time.sleep(1)

        assert environment_responses is not None, 'ERROR IN RETRY!'

        observations, observation_maps = self.make_observation(environment_responses)

        self.last_obs_maps = []
        self.last_reward_maps = []

        if not return_info:
            infos = {
                agent: {} for agent in self.possible_agents if agent in self.agents
            }
            return {agent: obs for agent, obs in zip(self.agents, observations)}, infos
        else:
            infos = {
                agent: {} for agent in self.possible_agents if agent in self.agents
            }
            return {agent: obs for agent, obs in zip(self.agents, observations)}, infos

    def step(self, action_dict):
        self.agents = [agent for agent in self.agents if not self.terminations[agent]]

        for vis in self.visuals:
            vis.publish()

        # if self.debug:
        #     print(f'Agents: {len(self.agents)}')

        actions = np.zeros(len(self.agents), dtype=np.int32)
        x_vels = np.zeros(len(self.agents), dtype=float)
        y_vels = np.zeros(len(self.agents), dtype=float)
        angle_vels = np.zeros(len(self.agents), dtype=float)
        for i, agent in enumerate(self.agents):
            if agent in action_dict:
                actions[i] = action_dict[agent]
                # actions[i] = -1
                x_vels[i] = 1.
                y_vels[i] = 1.
                angle_vels[i] = 1.

            if self.debug:
                actions[i] = 0

        total_rewards = [0. if len(self.last_reward_maps) == 0 else sum([v for v in (self.last_reward_maps[i].values() if len(self.last_reward_maps) > i else [0])]) for i in range(len(actions))]
        messages = [f'{i}' for i in range(len(actions))]

        if len(self.last_obs_maps) > 0:
            for idx, m in enumerate(self.last_obs_maps):
                if 'manual_zone_agent_zone_priority_order' in m and 'manual_zone_agent_zone_current_order' in m:
                    messages[idx] = f"{m['manual_zone_agent_zone_current_order'][0]} | {m['manual_zone_agent_zone_priority_order'][0]}"
                elif 'manual_zone_agent_zone_priority_order' in m:
                    messages[idx] = f"{m['manual_zone_agent_zone_priority_order'][0]}"


        # max_speeds = [min(max(1 / (x.get('manual_zone_agent_zone_priority_order', [-2.])[0] + 1), -1.), 1.) for x in self.last_obs_maps]
        # max_speeds = [min(max(2 - (x.get('manual_zone_agent_zone_priority_order', [0.])[0] * 0.5), -1.), 2.) for x in self.last_obs_maps]
        max_speeds = []
        if len(max_speeds) == 0:
            max_speeds = [-1.] * self.curr_num_agents
        environment_responses = self.sim_step([
            actions,
            x_vels,
            y_vels,
            angle_vels,
            max_speeds,
            messages,
            [AgentColor(reward=total_rewards[i]) for i in range(len(self.agents))]
        ])
        observations, observation_maps = self.make_observation(environment_responses)
        rewards, reward_maps = self.calculate_reward(observation_maps)

        self.last_obs_maps = observation_maps
        self.last_reward_maps = reward_maps

        # TODO - make a "done" observation
        # TODO - Difference between truncs and terms?
        agent_terminations = {agent: False if obs_map.get('success_observation', 0) == 0 else True for agent, obs_map in zip(self.agents, observation_maps)}

        # print(pprint.pformat(agent_terminations), flush=True)
        agent_observations = {agent: obs for (agent, obs) in zip(self.agents, observations)}
        # agent_rewards = {
        #     agent: reward if self.terminations_[idx] is False else 0 for idx, (agent, reward) in enumerate(zip(self.possible_agents, rewards)) if agent in self.agents
        # }
        agent_rewards = {
            agent: reward for idx, (agent, reward) in enumerate(zip(self.possible_agents, rewards)) if agent in self.agents
        }

        agent_infos = {
            agent: {
                'goal': obs_map.get('agents_goal'),
                'shortest_path': obs_map.get('agents_goal_distance'),
                'position': obs_map.get('agents_pose'),
                'succeeded': obs_map.get('success_observation', 0) == 1,
                'collision': obs_map.get('collisions', [0])[0],
                'velocity': obs_map.get('agents_velocity', np.array([0, 0])).mean(),
                'incorrect_enter_order': obs_map.get('entering_zone', [False])[0] and obs_map.get('manual_zone_agent_zone_current_order', [-1])[0] != obs_map.get('manual_zone_agent_zone_priority_order', [-1])[0],
                'incorrect_exit_order': obs_map.get('exiting_zone', [False])[0] and obs_map.get('manual_zone_agent_zone_current_order', [-1])[0] != obs_map.get('manual_zone_agent_zone_priority_order', [-1])[0],

            } for agent, obs_map in zip(self.possible_agents, observation_maps) if agent in self.agents
        }
        agent_infos["robot_data"] = agent_infos[self.agents[0]]
        agent_infos["pedestrian_data"] = agent_infos[self.agents[1]]
        agent_infos["pedestrian_data"] = [agent_infos["pedestrian_data"]] 
        agent_infos["collisions"] = 0
        agent_infos["success"] = False
        agent_infos["timestep"] = int(time.time())


        # if all([x.get('succeeded') for x in agent_infos.values()]):
        #     print('major reward')
        #     agent_rewards = {k: 100_000 for k in agent_rewards.keys()}

        self.terminations_ = list(agent_terminations.values())

        truncs = {agent: False if obs_map.get('success_observation', 0) == 0 else True for agent, obs_map in zip(self.agents, observation_maps)}
        #print(agent_infos, flush=True)

        # return agent_observations, agent_rewards, agent_terminations, truncs, agent_infos
        return agent_observations, agent_rewards, agent_terminations, truncs, agent_infos

    def render(self, mode="human"):
        """
        Depends on RVIZ for visualization, no render method
        """
        pass


class AgentColor:

    def __init__(self, r=-1., g=-1., b=-1., reward=None):
        self.r = r
        self.g = g
        self.b = b

        epsilon = 5

        if reward:
            self.r = 0 if reward >= 0 else 100 + min(abs(reward) * 10, 155)
            self.g = 0 if reward < 0 else 100 + min(abs(reward) * 10, 155)
            self.b = 0

        if reward is None or self.r + self.g + self.b < epsilon:
            self.r = -1.
            self.g = -1.
            self.b = -1.
