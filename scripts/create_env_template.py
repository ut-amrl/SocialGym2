import shutil
from argparse import ArgumentParser
from pathlib import Path
import subprocess
import json

SCRIPTS_FOLDER = Path(__file__).parent.absolute()
ROOT_FOLDER = SCRIPTS_FOLDER.parent
VECTOR_DISPLAY_FOLDER = ROOT_FOLDER / 'debugdocker/vectordisplay'
MAPS_FOLDER = VECTOR_DISPLAY_FOLDER / 'maps'
SRC_FOLDER = ROOT_FOLDER / 'src'
TEMPLATES_FOLDER = SRC_FOLDER / 'templates'
SUBMODULES_FOLDER = ROOT_FOLDER / 'submodules'
AMRL_MAPS_FOLDER = SUBMODULES_FOLDER / 'amrl_maps'
UT_MULTI_ROBOT_SIM_FOLDER = SUBMODULES_FOLDER / 'ut_multirobot_sim'
UT_MULTI_ROBOT_SIM_MAPS_FOLDER = UT_MULTI_ROBOT_SIM_FOLDER / 'maps'

def create_new_env(name: str, template: str = None):
    """
    Given a name and an optional template to copy from, use Vector Displays docker image to create a new vectormap and a
    new navigation map and store it under src/templates/{name}

    :param name: The name of the new environment, this is also the name of the folder the files will be saved under in
        the templates directory.
    :param template: The name of an existing environment to copy from (the original environment will not be touched, but
        the vector displays will open with their exact navigation and vectormap.)
    """

    environment_path = TEMPLATES_FOLDER / name

    if template:
        if environment_path.exists():
            shutil.rmtree(environment_path)

        shutil.copytree(str(TEMPLATES_FOLDER / template), str(environment_path))

    environment_path.mkdir(exist_ok=True, parents=True)

    environment_map_path = MAPS_FOLDER / name
    environment_map_path.mkdir(exist_ok=True, parents=True)

    vectormap_file = environment_map_path / f'{name}.vectormap.txt'
    vectormap_json_file = environment_path / f'{name}.vectormap.json'
    navigation_file = environment_map_path / f'{name}.navigation.txt'
    navigation_json_file = environment_map_path / f'{name}.navigation.json'

    if template:
        template_map_path = MAPS_FOLDER / template

        t_vectormap_file = template_map_path / f'{template}.vectormap.txt'
        t_navigation_file = template_map_path / f'{template}.navigation.txt'
        t_navigation_json_file = template_map_path / f'{template}.navigation.json'

        shutil.copyfile(t_navigation_file, navigation_file)
        shutil.copyfile(t_navigation_json_file, navigation_json_file)
        shutil.copyfile(t_vectormap_file, vectormap_file)



    if not vectormap_file.exists():
        open(str(vectormap_file), 'w').close()
    if not navigation_file.exists():
        open(str(navigation_file), 'w').close()

    # Build vectormap (geometry of the env)
    cmd = [str(VECTOR_DISPLAY_FOLDER / 'vd.sh'), str(MAPS_FOLDER), name, "--edit_localization"]

    out = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()

    # Build navigation paths (Navigation Graph)
    cmd = [str(VECTOR_DISPLAY_FOLDER / 'vd.sh'), str(MAPS_FOLDER), name, "--edit_navigation"]

    out = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()

    subprocess.Popen(["sudo", "chmod", "-R", "a+rwX",  f"{ROOT_FOLDER}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    subprocess.Popen(["sudo", "chmod", "-R", "a+rwX",  f"{AMRL_MAPS_FOLDER}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    subprocess.Popen(["sudo", "chmod", "-R", "a+rwX",  f"{UT_MULTI_ROBOT_SIM_MAPS_FOLDER}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


    # Build config files
    lf = launch_launch(name)
    hl = humans_lua()
    pl = pedsim_launch()
    sc = sim_config(name)
    s = scene()
    rl = ref_launch(name)
    pipsl = pips_launch(name)
    gl = greedy_launch(name)

    launch_file = environment_path / 'launch.launch'
    humans_file = environment_path / 'humans.lua'
    pedsim_file = environment_path / 'pedsim_launch.launch'
    sim_file = environment_path / 'sim_config.lua'
    scene_file = environment_path / 'scene.xml'
    ref_file = environment_path / 'ref_launch.launch'
    greedy_file = environment_path / 'greedy_launch.launch'
    pips_file = environment_path / 'pips_launch.launch'

    with launch_file.open('w') as f:
        f.write(lf)

    with humans_file.open('w') as f:
        f.write(hl)

    with pedsim_file.open('w') as f:
        f.write(pl)

    with sim_file.open('w') as f:
        f.write(sc)

    with scene_file.open('w') as f:
        f.write(s)

    with ref_file.open('w') as f:
        f.write(rl)

    with greedy_file.open('w') as f:
        f.write(gl)

    with pips_file.open('w') as f:
        f.write(pipsl)

    # Copy over maps
    shutil.copyfile(vectormap_file, environment_path / f'{name}.vectormap.txt')
    shutil.copyfile(navigation_file, environment_path / f'{name}.navigation.txt')
    shutil.copyfile(navigation_json_file, environment_path / f'{name}.navigation.json')

    vectormap_txt_to_json(vectormap_file, vectormap_json_file)

    amrl_folder = AMRL_MAPS_FOLDER / name
    amrl_folder.mkdir(exist_ok=True)

    shutil.copyfile(vectormap_file, amrl_folder / f'{name}.vectormap.txt')
    shutil.copyfile(vectormap_json_file, amrl_folder / f'{name}.vectormap.json')
    shutil.copyfile(navigation_file, amrl_folder / f'{name}.navigation.txt')
    shutil.copyfile(navigation_json_file, amrl_folder / f'{name}.navigation.json')

    utmulti_folder = UT_MULTI_ROBOT_SIM_MAPS_FOLDER / name
    utmulti_folder.mkdir(exist_ok=True)

    shutil.copyfile(vectormap_file, utmulti_folder / f'{name}.vectormap.txt')
    shutil.copyfile(vectormap_json_file, utmulti_folder / f'{name}.vectormap.json')
    shutil.copyfile(navigation_file, utmulti_folder / f'{name}.navigation.txt')
    shutil.copyfile(navigation_json_file, utmulti_folder / f'{name}.navigation.json')



def launch_launch(name: str):
    return \
f"""
<launch>
    <arg name="outfile" default="screen" />

    <node pkg="ut_multirobot_sim" type="simulator_link" name="simulator" cwd="node"
        output="screen"
        args="-sim_config $(find social_gym)/config/gym_gen/sim_config.lua -scene_config $(find social_gym)/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim" />

    <group unless="$(optenv DOCKER false)">
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ut_multirobot_sim)/visualization.rviz" />
    </group>

    <node pkg="graph_navigation" type="social_nav" name="graph_navigation" output="screen"
        args="-service_mode=true -map={name}"  />

    <include file="$(find social_gym)/config/gym_gen/pedsim_launch.launch" />

    <param name="enable_statistics" value="true" />
</launch>
"""


def humans_lua():
    return \
"""
function Vector2(x, y)
    return {x = x, y = y}
end

-- Human shape information
hu_radius = 0.2
hu_num_segments = 20

-- Human speed information
hu_max_speed = 0.8 --
hu_avg_speed = 0.8 --
hu_max_omega = 0.2
hu_avg_omega = 0.
hu_reach_goal_threshold = 0.1

-- Human walking mode
local HumanMode = {
    Singleshot=0,
    Repeat=1,
    Controlled=2,
    Cycle=3,
}

hu_mode = HumanMode.Controlled
hu_control_topic = "/command"

{% for i in range(human_count) %}
    hu{{ i }}_waypoints = {
        { {{ human_positions[i][0] }}, {{ human_positions[i][1] }}, {{ human_positions[i][2] }} }
    }
{% endfor %}
"""


def pedsim_launch():
    return \
f"""
<launch>
  <arg name="kbd_teleop" default="false"/>
  <arg name="rqt_teleop" default="false"/>
  <arg name="visualize" default="true"/>
  <arg name="with_robot" default="true"/>

  <arg name="simulation_factor" default="1.0"/> <!-- Speed up -->
  <arg name="update_rate" default="40.0"/> <!-- Hz -->

  <!-- Simulator -->
  <include file="$(find pedsim_simulator)/launch/simulator.launch">
    <arg name="kbd_teleop" value="$(arg kbd_teleop)"/>
    <arg name="rqt_teleop" value="$(arg rqt_teleop)"/>
    <arg name="scene_file" value="$(find social_gym)config/gym_gen/scene.xml"/>
    <arg name="with_robot" value="$(arg with_robot)"/>
    <arg name="simulation_factor" value="$(arg simulation_factor)"/>
    <arg name="update_rate" value="$(arg update_rate)"/>
    <arg name="default_queue_size" value="10"/>
    <arg name="max_robot_speed" value="1.5"/>
    <arg name="robot_mode" value="1"/>
    <arg name="enable_groups" value="false"/>
    <arg name="pose_initial_x" value="-1.45"/>
    <arg name="pose_initial_y" value="10.35"/>
    <arg name="pose_initial_theta" value="-1.5708"/>
  </include>

  <!-- <include file="$(find pedsim_visualizer)/launch/visualizer.launch"/> -->
  <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find pedsim_simulator)/rviz/social_contexts_activities.rviz" if="$(arg visualize)"/> -->

</launch>
"""


def sim_config(name):
    return \
"""
function Vector2(x, y)
    return {x = x, y = y}
  end

  function Vector3(x, y, z)
    return {x = x, y = y, z = z}
  end

  function DegToRad(d)
    return math.pi * d / 180
  end
""" + \
f"map_name =  \"maps/{name}/{name}.vectormap.txt\"" + \
"""
  -- Simulator starting location.
  start_poses = {
    {
      {{ robot_start[0] }},
      {{ robot_start[1] }},
      {{ robot_start[2] }}
    }
  }

  goal_poses = {
    {
      {{ robot_end[0] }},
      {{ robot_end[1] }},
      {{ robot_end[2] }}
    }
  }

  num_humans = {{ human_count }}
  human_config = "../../config/gym_gen/humans.lua"

  door_config_list = {
    -- "/home/jaholtz/code/amrl_maps/GDC1/door_list.lua"
  }

  -- Time-step for simulation.
  delta_t = 0.025
  -- max_steps = 12000
  max_steps = 3200

  -- Simulator TF publications
  publish_tfs = true;
  -- publish_foot_to_base = true;
  -- publish_map_to_odom = true;

  -- -- Car dimensions.
  -- car_width = 0.281
  -- car_length = 0.535
  -- car_height = 0.15;

  -- -- Location of the robot's rear wheel axle relative to the center of the body.
  -- rear_axle_offset = -0.162
  -- laser_loc = Vector3(0.2, 0, 0.15)

  -- Kinematic and dynamic constraints for the car.
  -- min_turn_radius = 0.98

  -- Laser rangefinder parameters.
  laser_noise_stddev = 0.01;
  -- laser_angle_min = DegToRad(-135.0);
  -- laser_angle_max = DegToRad(135.0);
  -- laser_angle_increment = DegToRad(0.25);
  -- laser_min_range = 0.02;
  -- laser_max_range = 100.0;

  -- Turning error simulation.
  angular_error_bias = DegToRad(0);
  angular_error_rate = 0.1;

  -- Defining robot type enumerator
  local RobotType = {
      ACKERMANN_DRIVE="ACKERMANN_DRIVE",
      OMNIDIRECTIONAL_DRIVE="OMNIDIRECTIONAL_DRIVE",
      DIFF_DRIVE="DIFF_DRIVE"
  }

  -- robot_type = RobotType.ACKERMANN_DRIVE
  -- robot_config = "config/ut_automata_config.lua"
  -- robot_type = RobotType.DIFF_DRIVE
  -- robot_config = "config/bwibot_config.lua"
  -- robot_type = RobotType.OMNIDIRECTIONAL_DRIVE
  -- robot_config = "config/cobot_config.lua"
  robot_types = {
    RobotType.DIFF_DRIVE
  }
  robot_config = "config/ut_jackal_config.lua"

  laser_topic = "/Cobot/Laser"
  laser_frame = "base_laser"
  """


def scene():
    return \
"""
<?xml version="1.0" encoding="UTF-8"?>
<scenario>
    <!--Obstacles-->
    <obstacle x1="-1.014007" y1="-0.003961" x2="-1.014007" y2="-0.003961"/>
	<obstacle x1="-1.742824" y1="-3.980769" x2="-0.483238" y2="1.010046"/>
	<obstacle x1="-0.499081" y1="1.017968" x2="0.499081" y2="1.025890"/>
	<obstacle x1="0.499081" y1="1.025890" x2="1.164523" y2="-4.258036"/>
	<obstacle x1="-1.774608" y1="-3.950327" x2="-5.737090" y2="-8.058667"/>
	<obstacle x1="-5.761400" y1="-8.107286" x2="0.486194" y2="-12.483033"/>
	<obstacle x1="0.486194" y1="-12.483033" x2="6.223284" y2="-6.575775"/>
	<obstacle x1="6.223284" y1="-6.575775" x2="1.191175" y2="-4.242043"/>    
    <!--Way Points-->
{% for i in range(position_count) %}
    <waypoint id="{{ i }}" x="{{ positions[i][0] }}" y = "{{ positions[i][1] }}" r="1" b="0.1"/>
{% endfor %}

{% for i in range(nav_count) %}
    <waypoint id="n{{ i }}" x="{{ nav_map[i][0] }}" y = "{{ nav_map[i][1] }}" r="1" b="0.1"/>
{% endfor %}

    <!-- This Robot Goal Doesn't Matter, but is Required -->
  <waypoint id="robot_goal" x="22" y="27" r="2"/>
  <waypoint id="robot_start" x="{{ robot_start[0] }}" y="{{ robot_start[1] }}" r="2"/>

  <agent x="{{ robot_start[0] }}" y="{{ robot_start[1] }}" n="1" dx="0" dy="0" type="2">
    <addwaypoint id="robot_start"/>
    <addwaypoint id="robot_goal"/>
  </agent>

  {% for human_position in human_positions %}
  <agent x="{{ human_position[0] }}" y="{{ human_position[1] }}" n="1" dx="{{ dev }}" dy="{{ dev }}" type="0">
    {% for i in range(3, human_position|length) %}
    <addwaypoint id="n{{human_position[i]}}" />
    {% endfor %}
  </agent>
  {% endfor %}

</scenario>
"""


def ref_launch(name):
    return \
f"""
<launch>
    <arg name="outfile" default="screen" />
    <arg name="docker" default="false"/>

    <node pkg="ut_multirobot_sim" type="simulator_link" name="simulator" cwd="node"
        output="screen"
        args="-sim_config $(find ut_multirobot_sim)/config/gym_gen/sim_config.lua -scene_config $(find ut_multirobot_sim)/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim" />

    <group unless="$(optenv DOCKER false)">
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ut_multirobot_sim)/visualization.rviz" />
    </group>

    <node pkg="pips" type="social_ref" name="social_ref" output="screen" />

    <node pkg="graph_navigation" type="social_nav" name="graph_navigation" output="screen"
        args="-service_mode=true -social_mode=true -map={name}" />

    <include file="$(find ut_multirobot_sim)/config/gym_gen/pedsim_launch.launch" />

    <param name="enable_statistics" value="true" />
</launch>
"""


def greedy_launch(name):
    return \
f"""
<launch>
    <arg name="outfile" default="screen" />

    <node pkg="ut_multirobot_sim" type="simulator_link" name="simulator" cwd="node"
        output="screen"
        args="-sim_config $(find ut_multirobot_sim)/config/gym_gen/sim_config.lua -scene_config $(find ut_multirobot_sim)/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim" />

    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ut_multirobot_sim)/visualization.rviz" />

    <node pkg="cpp-pips" type="social_greed" name="social_greed" output="screen" />

    <node pkg="graph_navigation" type="social_nav" name="graph_navigation" output="screen"
        args="-service_mode=true -social_mode=true -map={name}" />

    <include file="$(find ut_multirobot_sim)/config/gym_gen/pedsim_launch.launch" />

    <param name="enable_statistics" value="true" />
</launch>
"""


def pips_launch(name):
    return \
f"""
<launch>
    <arg name="docker" default="false"/>
    <arg name="outfile" default="screen" />

    <node pkg="ut_multirobot_sim" type="simulator_link" name="simulator" cwd="node"
        output="screen"
        args="-sim_config $(find ut_multirobot_sim)/config/gym_gen/sim_config.lua -scene_config $(find ut_multirobot_sim)/config/gym_gen/scene.xml -speedup_factor 1.0 --localize --use_pedsim" />

    <group unless="$(optenv DOCKER false)">
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ut_multirobot_sim)/visualization.rviz" />
    </group>

    <node pkg="pips" type="social_interp" name="social_interp" output="screen" args="-ast_path $(find cpp-pips)/synthd/displ3" />

    <node pkg="graph_navigation" type="social_nav" name="graph_navigation" output="screen"
        args="-service_mode=true -social_mode=true -map={name}" />

    <include file="$(find ut_multirobot_sim)/config/gym_gen/pedsim_launch.launch" />

    <param name="enable_statistics" value="true" />
</launch>
"""


def vectormap_txt_to_json(
        vectormap_text_file: Path,
        vectormap_json_output_file: Path,
):
    """
    Helper function for turning the txt formatted vectormap files into json vector maps (TODO - is this needed?)

    :param vectormap_text_file: The path object to a x.vectormap.txt file.
    :param vectormap_json_output_file: Where you want to store the output vectormap.json file.
    """

    assert vectormap_text_file.name.endswith('.vectormap.txt'), 'please specify a correct {x}.vectormap.txt file'
    assert vectormap_json_output_file.name.endswith('.vectormap.json'), \
        'please specify a correct {x}.vectormap.json file'

    vectors = []

    def clean_point(point: str) -> str:
        return point.strip().replace(',', '')

    with vectormap_text_file.open('r') as r:
        lines = r.readlines()
        for line in lines:
            values = line.strip().split(" ")
            p0x, p0y, p1x, p1y = values

            p0x = clean_point(p0x)
            p0y = clean_point(p0y)
            p1x = clean_point(p1x)
            p1y = clean_point(p1y)

            vectors.append({'p0': {'x': p0x, 'y': p0y}, 'p1': {'x': p1x, 'y': p1y}})

    with vectormap_json_output_file.open('w') as w:
        json.dump(vectors, w)


if __name__ == "__main__":
    argparser = ArgumentParser()

    argparser.add_argument('--name', '-n', type=str, help='Name of the environment', required=True)
    argparser.add_argument('--template', '-t', type=str, help='Name of an existing environment to copy from')

    args = argparser.parse_args()

    name: str = args.name
    template: str = args.template

    create_new_env(name, template)
