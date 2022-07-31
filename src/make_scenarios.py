#!/usr/bin/env python3

import os
import sys
from random import seed
from random import randint
import math
import os
import jinja2
import random
import rospy
import roslib
import json
NODE_NAME = 'ros_social_gym'
roslib.load_manifest(NODE_NAME)
from graph_navigation.srv import graphNavSrv
from amrl_msgs.msg import Pose2Df
from shutil import copyfile
from pathlib import Path

config_nav_path = 'src/templates/narrowtest/narrowtest.navigation.json'
config_scene_path = 'src/templates/narrowtest/'

test = Path(config_scene_path).absolute()
test1 = Path(config_nav_path).absolute()

print(test1)
print(test)

print(test1.exists())
print(test1.is_file())
# CMU Scenario
#  config_nav_path = 'maps/GHC3/GHC3.navigation.json'
#  config_scene_path = 'src/ros_social_gym/templates/GHC3/'

# Format: (x, y, theta)
robot_positions = [
    (34.3, 7.04, 1.5708),
    (36.04, 9.06, 3.1459),
    (36.124, 15.972, 3.14),
    (29.15, 15.69, -1.5708),
    (23.49, 6.68, 1.5708),
    (2.03, 6.68, 1.5708),
    (-6.38, 6.38, 1.5708),
    (-10.06,6.36, 1.5708),
    (-13.68, 6.36, 1.5708),
    (-19, 6.36, 1.5708),
    (-30.05, 4.27, 0),
    (-30.05, 13.95, 0),
    (-38.1, 15.8, 0),
    (-36.1, 18.18, -1.5708),
    (-36.1, 20.65, 0),
    (-24.39, 16.1, 3.14),
    (-20.78, 10.35, -1.5708),
    (-14.38, 10.35, -1.5708),
    (-1.45, 10.35, -1.5708),
    (0.74, 10.35, -1.5708),
    (9.9, 10.35, -1.5708),
    (15.09, 10.35, -1.5708),
    (19.75, 16.35, 3.14),
    (14.03, 24.11, 0),
    (-1.6, 16.24, -1.5708),
    (-13.71, 16.24, 0)
]

nav_map = [
    (38.394, 19.271),
    (28.734, 19.777),
    (29.069, 8.621),
    (9.770, 8.735),
    (15.182, 6.621),
    (14.954, 8.704),
    (14.836, 12.641),
    (39.381, 21.205),
    (36.060, 9.116),
    (34.291, 8.973),
    (34.234, 7.491),
    (32.790, 8.820),
    (32.742, 7.309),
    (23.622, 8.619),
    (23.660, 7.109),
    (21.470, 8.686),
    (21.499, 7.233),
    (19.616, 8.877),
    (19.683, 10.655),
    (13.841, 7.013),
    (13.822, 8.705),
    (9.893, 10.780),
    (7.503, 6.783),
    (7.531, 8.552),
    (6.097, 8.581),
    (6.145, 7.003),
    (2.340, 8.676),
    (2.311, 7.128),
    (0.715, 10.187),
    (0.734, 8.648),
    (-1.496, 10.413),
    (-0.324, 10.437),
    (-1.393, 11.857),
    (-0.116, 11.841),
    (-1.454, 12.744),
    (-3.043, 12.608),
    (-3.009, 14.418),
    (-3.001, 11.036),
    (-1.710, 15.093),
    (-1.700, 16.353),
    (-1.607, 18.646),
    (-13.858, 19.090),
    (-13.889, 15.868),
    (-13.827, 12.924),
    (-13.827, 8.461),
    (-13.703, 6.581),
    (-10.046, 8.182),
    (-10.087, 6.220),
    (-6.431, 8.244),
    (-6.410, 6.540),
    (-4.799, 8.420),
    (-4.768, 7.118),
    (-18.940, 8.172),
    (-18.909, 6.292),
    (-20.737, 8.554),
    (-20.685, 10.538),
    (-20.613, 16.374),
    (-24.559, 16.208),
    (-26.563, 16.095),
    (-26.708, 13.967),
    (-30.054, 14.060),
    (-28.210, 18.521),
    (-34.461, 20.710),
    (-36.486, 20.754),
    (-35.807, 15.948),
    (-35.862, 17.656),
    (-38.216, 15.817),
    (-26.991, 4.441),
    (-29.759, 4.355),
    (-36.514, 4.542),
    (-36.815, 14.137),
    (-13.681, 4.700),
    (-9.952, 4.614),
    (-1.347, 8.672),
    (13.836, 21.237),
    (13.859, 24.275),
    (13.955, 30.517)
]

def LoadNavNodes(nav_path):
    temp_nav = []
    with open(nav_path, 'r') as input:
        json_list = json.load(input)
        for entry in json_list['nodes']:
            point = (float(entry['loc']['x']), float(entry['loc']['y']), 0.0)
            temp_nav.append(point)
    return list(set(temp_nav))

def MakeScenario(config):
    dir_name = "config/gym_gen/"

    with open(config_scene_path + 'humans.lua', 'r') as f:
        human_lua_template = jinja2.Template(f.read())

    with open(config_scene_path + 'sim_config.lua', 'r') as f:
        sim_config_lua_template = jinja2.Template(f.read())

    with open(config_scene_path + 'launch.launch', 'r') as f:
        launch_template = jinja2.Template(f.read())

    with open(config_scene_path + 'pedsim_launch.launch', 'r') as f:
        pedsim_launch_template = jinja2.Template(f.read())

    with open(config_scene_path + 'scene.xml', 'r') as f:
        scene_xml_template = jinja2.Template(f.read())

    # Create config directory
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)

    # Make launch file
    with open(dir_name + 'launch.launch', 'w') as f:
        f.write(launch_template.render(config))

    # Make pedsim launch file
    with open(dir_name + 'pedsim_launch.launch', 'w') as f:
        f.write(pedsim_launch_template.render(config))

    # Make Humans File
    with open(dir_name + 'humans.lua', 'w') as f:
        f.write(human_lua_template.render(config))

    # Creates the scene xml file
    with open(dir_name + 'scene.xml', 'w') as f:
        f.write(scene_xml_template.render(config))

    # creates the simulator config file
    with open(dir_name + 'sim_config.lua', 'w') as f:
        f.write(sim_config_lua_template.render(config))

    copyfile(config_scene_path + 'ref_launch.launch', dir_name + 'ref_launch.launch')
    copyfile(config_scene_path + 'greedy_launch.launch', dir_name + 'greedy_launch.launch')
    copyfile(config_scene_path + 'pips_launch.launch', dir_name + 'pips_launch.launch')

def GenerateScenario():
    global robot_positions
    global nav_map
    if (config_nav_path != 'GDC'):
        nav_map = LoadNavNodes(config_nav_path)
        robot_positions = nav_map
    num_humans = randint(1, 1)
    robot_start = randint(0, len(robot_positions) - 1)
    robot_end = robot_start
    while (robot_start == robot_end):
        robot_end = randint(0, len(robot_positions) - 1)
    human_positions = []
    for i in range(0, num_humans):
        human_start = randint(0, len(robot_positions) - 1)
        while (human_start == robot_start):
            human_start = randint(0, len(robot_positions) - 1)
        human_end = human_start
        while (human_start == human_end):
            human_end = randint(0, len(robot_positions) - 1)
        planner = rospy.ServiceProxy('graphNavSrv', graphNavSrv)
        rospy.wait_for_service('graphNavSrv')
        h_start = robot_positions[human_start]
        h_end = robot_positions[human_end]
        start = Pose2Df(h_start[0], h_start[1], 0)
        end = Pose2Df(h_end[0], h_end[1], 0)
        resp = planner(start, end)
        print(f"RAW PLAN: {resp}")
        plan = [x for x in resp.plan if x < len(nav_map)]
        r_plan = plan[::-1]
        human_list = [robot_positions[human_start][0],
                      robot_positions[human_start][1],
                      human_end]
        human_list.extend(r_plan)
        human_list.extend(plan)
        human_positions.append(human_list)
    # Build the Config Dictionary
    human_dev =  1.0
    config = {
        'robot_start': robot_positions[robot_start],
        'robot_end': robot_positions[robot_end],
        'human_count': num_humans,
        'position_count': len(robot_positions),
        'nav_count': len(nav_map),
        'dev': human_dev,
        'positions': robot_positions,
        'human_positions': human_positions,
        'nav_map': nav_map
    }
    MakeScenario(config)


if __name__ == "__main__":
    GenerateScenario()