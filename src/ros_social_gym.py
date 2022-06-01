# Ros Imports
import roslib
NODE_NAME = 'social_gym'
roslib.load_manifest(NODE_NAME)
from ut_multirobot_sim.srv import utmrsStepper
from ut_multirobot_sim.srv import utmrsReset
from amrl_msgs.srv import SocialPipsSrv
from ut_multirobot_sim.srv import utmrsStepperResponse
from make_scenarios import GenerateScenario
from shutil import copyfile
import rospy
import roslaunch

# Other Imports
import copy
import sys
import gym
from gym import spaces
import json
import numpy as np
import os
import time
import math
from statistics import mean
from random import seed

def MakeNpArray(poseList):
  coordList = []
  for pose in poseList:
    coordList.append(pose.x)
    coordList.append(pose.y)
    coordList.append(pose.theta)
  return np.array(coordList)

def ClosestHuman(robot_pose, poses):
  best_dist = 9999
  best_pose = robot_pose
  best_index = 0
  index = 0
  for pose in poses:
    pose_array = np.array([pose.x, pose.y])
    if (np.linalg.norm(pose_array) == 0):
      continue
    distance = np.linalg.norm(robot_pose - pose_array)**2
    if (distance < best_dist):
      best_index = index
      best_dist = distance
      best_pose = pose_array
    index += 1
  return best_pose, best_dist, best_index

def Force(response):
  robot_pose = response.robot_poses[0]
  robot_pose = np.array([0, 0])
  human_poses = copy.deepcopy(response.human_poses)
  if (response.robot_state == 2):
      closest_index = ClosestHuman(robot_pose, human_poses)[2]
      del human_poses[closest_index]
  # Find the closest human to the robot
  closest_distance = ClosestHuman(robot_pose, human_poses)[1]
  if (closest_distance == 9999):
    return 0
  force = np.exp(-closest_distance**2)
  return force

def sigmoid(x):
  return 1 - math.erf(x)

def closest_point_on_line_segment_to_point(end1, end2, point):
  l2 = np.linalg.norm(end1 - end2)**2
  if np.abs(l2) < 1e-6:
    return end1
  t = max(0, min(1, np.dot(point - end1, end2 - end1) / l2))
  projection = end1 + t * (end2 - end1)
  return projection

def Blame(response):
  # Find the closest human
  robot_pose = response.robot_poses[0]
  robot_pose = np.array([0, 0])
  # if state is halt, blame = 0 (we have no velocity anymore)
  if (response.robot_state == 1):
    return 0.0
  # If following don't count blame on follow target.
  human_poses = copy.deepcopy(response.human_poses)
  if (response.robot_state == 2):
      closest_index = ClosestHuman(robot_pose, human_poses)[2]
      del human_poses[closest_index]
  human, closest_distance, index = ClosestHuman(robot_pose, human_poses)
  if (closest_distance == 9999):
      return 0.0

  # forward predicted robot position
  robot_vel = response.robot_vels[0]
  robot_vel = [robot_vel.x, robot_vel.y]
  if (np.linalg.norm(robot_vel)) < 1e-6:
    return 0
  end2 = robot_pose + (np.array(robot_vel) * 0.5)

  # closest point to human
  closest_point = closest_point_on_line_segment_to_point(robot_pose, end2, human)

  # blame
  blame = sigmoid(np.linalg.norm(closest_point - human))
  return blame

def DistanceFromGoal(response):
  robot_pose = MakeNpArray(response.robot_poses)
  goal_pose = MakeNpArray([response.goal_pose])
  return np.linalg.norm(robot_pose[0] - goal_pose[0])

def np_encoder(object):
  if isinstance(object, np.generic):
    return object.item()

class RosSocialEnv(gym.Env):
  """A ros-based social navigation environment for OpenAI gym"""

  def __init__(self, reward, repeats, launch):
    super(RosSocialEnv, self).__init__()
    # Halt, GoAlone, Follow, Pass
    self.action_space = spaces.Discrete(4)
    self.rewardType = reward
    # goal_x, goal_y, robot_1x, robot_1y, ... ,
    # robot_nx, robot_ny, robot_1vx, robot_1vy, ...,
    # human_1x, human_1y, ..., human_1vx, human_1vy ...
    # next_door_x, next_door_y, next_door_state
    # Observation Parameters
    self.num_robots = 1
    self.max_humans = 40
    self.noPose = True
    self.length = 1 + (self.num_robots*6) + (self.max_humans * 6)
    if self.noPose:
      self.length = 4 + (self.max_humans * 6)
      #  self.length = 1 + (self.num_robots*3) + (self.max_humans * 6)
    self.observation_space = spaces.Box(low=-9999,
                                        high=9999,
                                        shape=(self.length,))

    # Initialize Ros Here
    rospy.init_node('RosSocialGym', anonymous=True)
    # Launch the simulator launch file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch])
    self.launch.start()
    rospy.wait_for_service('utmrsStepper')
    rospy.wait_for_service('utmrsReset')
    self.simStep = rospy.ServiceProxy('utmrsStepper', utmrsStepper)
    self.simReset = rospy.ServiceProxy('utmrsReset', utmrsReset)
    self.pipsSrv = rospy.ServiceProxy('SocialPipsSrv', SocialPipsSrv)
    #  GenerateScenario()

    # Initialize Internal Variables
    self.action = 0
    self.numRepeats = repeats
    self.last_observation = []
    self.startDist = 1.0
    self.lastDist = 1.0
    self.resetCount = 0
    self.stepCount = 0
    self.totalSteps = 0
    self.data = {
      'Iteration': 0,
      'NumHumans': 0,
      'Success': 0,
      'Collision': 0,
      'Steps': 0,
      'Data': []
    }
    self.lastObs = utmrsStepperResponse()

  # Do this on shutdown
  def __del__(self):
    self.launch.shutdown()

  def MakeObservation(self, res):
    obs = []
    if (self.noPose):
      pass
    else:
      obs = MakeNpArray(res.robot_poses)
      obs = np.append(obs, MakeNpArray(res.robot_vels))
    fill_num = ((self.max_humans) - (len(res.human_poses)))
    self.data["NumHumans"] = len(res.human_poses)
    fill_num = 3 * fill_num
    obs = np.append(obs, MakeNpArray(res.human_poses))
    obs = np.append(obs, np.zeros(fill_num))
    obs = np.append(obs, MakeNpArray(res.human_vels))
    obs = np.append(obs, np.zeros(fill_num))
    # Pad with zeroes to reach correct number of variables
    obs = np.append(obs, MakeNpArray([res.local_target]))
    obs = np.append(obs, self.action)
    return obs

  def CalculateReward(self, res):
    distance = DistanceFromGoal(res)
    score = (self.lastDist - distance)
    self.lastDist = distance
    force = Force(res)
    blame = Blame(res)
    bonus = 100.0 if res.success else 0.0
    penalty = -1.0 if res.collision else 0.0
    if (self.rewardType == '0'): # No Social
      return (10 * score) + bonus
    elif (self.rewardType == '1'): # Nicer
      w1 = 5.0
      w2 = -0.1
      w3 = -0.01
      if (score < 0.0):
        w1 *= 2.0
      cost = w1 * score + w2 * blame + w3 * force
      return cost + bonus
    elif (self.rewardType == '2'): # Greedier
      w1 = 10.0
      w2 = -0.05
      w3 = -0.01
      cost = w1 * score + w2 * blame + w3 * force
      return cost + bonus
    return score + bonus

  def reset(self):
    # Reset the state of the environment to an initial state
    # Call the "reset" of the simulator
    self.resetCount += 1
    self.totalSteps += self.stepCount
    self.stepCount = 0
    kNumRepeats = self.numRepeats
    # Create a new scenario if necessary
    #  if (self.resetCount % kNumRepeats == 0):
      #  GenerateScenario()
    response = self.simReset()
    stepResponse = self.simStep(0)
    self.startDist = DistanceFromGoal(stepResponse)
    self.lastDist = self.startDist
    self.lastObs = stepResponse

    return self.MakeObservation(stepResponse)

  def step(self, action):
    self.stepCount += 1
    self.data["Steps"] = self.stepCount
    tic = time.perf_counter()
    # Execute one time step within the environment
    # Call the associated simulator service (with the input action)
    self.action = action
    if (self.action != self.lastObs.robot_state):
      print("Transition TO: " + str(self.action))

    # Step the simulator and make observation
    response = self.simStep(action)
    self.lastObs = response
    toc = time.perf_counter()
    obs = self.MakeObservation(response)
    obs = [0 if math.isnan(x) else x for x in obs]

    # Calculate a Reward
    reward = self.CalculateReward(response)
    self.data["Reward"] = reward
    done = response.done
    if (response.success):
      self.data["Success"] = 1
    if (response.collision):
      self.data["Collision"] += 1

    return obs, reward, done, {"resetCount" : self.resetCount}

  # Depends on RVIZ for visualization, no render method
  def render(self):
    print(f'Render')
