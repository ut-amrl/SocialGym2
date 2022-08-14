#!/usr/bin/env python3

import os
import sys
from random import seed
from random import randint
import math
import os
import jinja2
import random
import json

from shutil import copyfile

from pathlib import Path
from typing import Union, Tuple


class Scenario:

    def __init__(self, env_name: str = None):
        if env_name is not None:
            self.config_nav_path = Path(f'src/templates/{env_name}/{env_name}.navigation.json')
            self.config_scene_path = Path(f'src/templates/{env_name}/')

            assert Path(self.config_nav_path).is_file(), \
                f'The env_name ({env_name}) does not have a navigation file at {self.config_nav_path}.'
            assert Path(self.config_scene_path).is_dir(), \
                f'The env_name ({env_name}) does not have a folder at {self.config_scene_path}'
        else:
            self.config_nav_path = None
            self.config_scene_path = Path('src/templates/gdc/')


    def load_nav_nodes(self, nav_path):
        temp_nav = []
        with open(nav_path, 'r') as input:
            json_list = json.load(input)
            for entry in json_list['nodes']:
                point = (float(entry['loc']['x']), float(entry['loc']['y']), 0.0)
                temp_nav.append(point)
        return list(set(temp_nav))

    def make_scenario(self, config):
        dir_name = "config/gym_gen/"

        with (self.config_scene_path / 'humans.lua').open('r') as f:
            human_lua_template = jinja2.Template(f.read())

        with (self.config_scene_path / 'sim_config.lua').open('r') as f:
            sim_config_lua_template = jinja2.Template(f.read())

        with (self.config_scene_path / 'launch.launch').open('r') as f:
            launch_template = jinja2.Template(f.read())

        with (self.config_scene_path / 'pedsim_launch.launch').open('r') as f:
            pedsim_launch_template = jinja2.Template(f.read())

        with (self.config_scene_path / 'scene.xml').open('r') as f:
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

        copyfile(str(self.config_scene_path / 'ref_launch.launch'), dir_name + 'ref_launch.launch')
        copyfile(str(self.config_scene_path / 'greedy_launch.launch'), dir_name + 'greedy_launch.launch')
        copyfile(str(self.config_scene_path / 'pips_launch.launch'), dir_name + 'pips_launch.launch')

    def generate_scenario(self):
        raise NotImplemented()
