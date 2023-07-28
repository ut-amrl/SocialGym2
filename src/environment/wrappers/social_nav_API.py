import gym
from gym import Wrapper, Env
import numpy as np
from typing import Union, Tuple, Dict
from pettingzoo.utils.wrappers import BaseParallelWrapper
import pprint
      

GymObs = Union[Tuple, Dict, np.ndarray, int]

class SocialNavWrapper(BaseParallelWrapper):
    def __init__(self, env: gym.Env, metrics):
      super().__init__(env)
    #   self.env = env
      self.metrics = [metric(self) for metric in metrics]

      self.time = 0
      self.steps = 0
      self.history = []

    @property
    def state(self):
      return self.history[-1] if self.history else None
    
    def reset(self, seed=None, return_info=False, options=None):
        self.episode_steps = 0
        res = self.env.reset(seed=seed, options=options)
        self.agents = self.unwrapped.agents
        return res

    def step(self, action: Union[int, np.ndarray]) -> Tuple[GymObs, float, Dict[str, bool], Dict[str, bool], Dict]:
        obs, reward, done, truncs, infos = self.env.step(action)
        self.agents = self.unwrapped.agents
        print(pprint.pformat(infos), flush=True)
        self.parse_info(infos)
        return obs, reward, done, truncs, infos

    def parse_info(self, info):
      self.history.append(State.create_from_info(info))
      self.time += self.state.timestep
      self.steps += 1

    def get_metrics(self):
      metric_values = {}
      for metric in self.metrics:
        metric_values.update(metric.compute())
      return metric_values


#-----------------------------------------------------
# State and info parsing
#-----------------------------------------------------
class State:
  def __init__(self,
               robot,
               pedestrians,
              #  obstacles,
               collisions,
               success,
               timestep):
    self.robot = robot
    self.pedestrians = pedestrians
    # self.obstacles = obstacles
    self.collisions = collisions
    self.success = success
    self.timestep = timestep

  @property
  def min_distance_to_other_agents(self):
    robot_position = self.robot.position
    return min([
        np.linalg.norm(robot_position - pedestrian.position)
        for pedestrian in self.pedestrians
        ])

  @classmethod
  def create_from_info(cls, info):
    robot = parse_robot(info)
    pedestrians = parse_pedestrians(info)
    # obstacles = parse_obstacles(info)
    collisions = parse_collisions(info)
    success = parse_success(info)
    timestep = parse_timestep(info)
    return cls(
        robot=robot,
        pedestrians=pedestrians,
        # obstacles=obstacles,
        collisions=collisions,
        success=success,
        timestep=timestep)

def parse_pedestrians(info):
    pedestrian_data = info["pedestrian_data"]
    pedestrians = []
    for data in pedestrian_data:
        position = data["position"]
        velocity = data["velocity"]
        goal = data["goal"]
        pedestrians.append(Pedestrian(position, velocity, goal))
    return pedestrians

def parse_robot(info):
    robot_data = info["robot_data"]
    return Robot(
        position=robot_data["position"],
        velocity=robot_data["velocity"],
        goal=robot_data["goal"],
        shortest_path=robot_data["shortest_path"]
    )

def parse_obstacles(info):
    obstacle_data = info["obstacle_data"]
    obstacles = []
    for data in obstacle_data:
        points = data["points"]
        obstacles.append(Obstacle(points))
    return obstacles

def parse_goal(info):
    return info["robot_data"]["goal"]

def parse_collisions(info):
    return info["collisions"]

def parse_success(info):
    return info["success"]

def parse_timestep(info):
    return info["timestep"]

#-----------------------------------------------------
# Robot, pedestrians and obstacles
#-----------------------------------------------------
class Pedestrian:
  def __init__(self, position, velocity, goal):
    self.position = np.array(position)
    self.velocity = np.array(velocity)
    self.goal = np.array(goal)

class Robot:
  def __init__(self, position, velocity, goal, shortest_path):
    self.position = np.array(position)
    self.velocity = np.array(velocity)
    self.goal = np.array(goal)
    self.shortest_path = shortest_path

class Obstacle:
    def __init__(self, points):
        self.poly = geometry.Polygon([[p[0], p[1]] for p in points])


#-----------------------------------------------------
# Metric Classes
#-----------------------------------------------------
class Metric:

  def __init__(self, env: gym.Env, name, keys=None):
    self.name = name
    # self.env = env
    self.keys = keys

  def compute(self):
    if self.keys:
      return {
          f"{self.name}_{key}": value
          for key, value in zip(self.keys, self.value(self.env))
      }
    else:
      return {self.name: self.value(self.env)}

  def value(self, env):
    pass

class SuccessMetric(Metric):

  def __init__(self, env: gym.Env, name="success"):
    super().__init__(env, name)

  def value(self, env):
    return env.state.success

class DistanceToGoalMetric(Metric):

  def __init__(self, env: gym.Env, name="distance_to_goal"):
    super().__init__(env, name)

  def value(self, env):
    return np.linalg.norm(env.state.robot.position - env.state.robot.goal)


class CollisionsMetric(Metric):

  def __init__(self, env: gym.Env, name="collisions"):
    super().__init__(env, name)

  def value(self, env):
    return env.state.collisions


class SplMetric(Metric):

  def __init__(self, env: gym.Env, name="spl"):
    super().__init__(env, name)

  def value(self, env):
    if not env.state.success:
      return 0
    shortest_path = env.state.robot.shortest_path
    traversed_distance = traversed_distance(env.history)
    return shortest_path / max(shortest_path, traversed_distance)

class MinDistanceToOtherAgentsMetric(Metric):

  def __init__(self, env: gym.Env, name="min_distance_to_other_agents"):
    super().__init__(env, name)

  def value(self, env):
    return min_distance_to_other_agents(env.history)


class JerkMetric(Metric):

  def __init__(self, env: gym.Env, name="jerk", keys=("min", "max", "avg", "sum")):
    super().__init__(env, name, keys)

  def value(self, env):
    return get_robot_jerk_stats(env.history)

STANDARD_METRICS = [
    SplMetric,
    SuccessMetric,
    CollisionsMetric,
    DistanceToGoalMetric,
    MinDistanceToOtherAgentsMetric,
    JerkMetric
]

#-----------------------------------------------------
# Support Library
#-----------------------------------------------------
def min_distance_to_other_agents(history):
  return min([state.min_distance_to_other_agents for state in history])

def get_robot_positions(history):
  return np.array([state.robot.position for state in history])

def traversed_distance(history):
  robot_positions = get_robot_positions(history)
  delta_positions = np.diff(robot_positions, axis=0)
  delta_distance = np.linalg.norm(delta_positions, axis=1)
  return sum(delta_distance)

def get_time_deltas(history):
  return np.array([state.timestep for state in history])

def get_robot_velocities(history):
  return np.array([state.robot.velocity for state in history])

def get_robot_jerk_history(history):
  velocities = get_robot_velocities(history)
  velocity_deltas = np.diff(velocities, axis=0)

  time_deltas = get_time_deltas(history)
  time_broadcast = np.broadcast_to(
      np.expand_dims(time_deltas[:-1], 1),
      velocity_deltas.shape)

  accelerations = velocity_deltas / time_broadcast
  acceleration_deltas = np.diff(accelerations, axis=0)

  jerk_history = acceleration_deltas / time_broadcast[:-1]
  jerk_magnitudes = np.linalg.norm(jerk_history, axis=1)

  return jerk_magnitudes, time_deltas[:-2]

def get_robot_jerk_stats(history):
  jerks, deltas = get_robot_jerk_history(history)
  if len(jerks) == 0:
    return 0, 0, 0, 0
  return jerks.min(), jerks.max(), jerks.mean(), sum(jerks * deltas)