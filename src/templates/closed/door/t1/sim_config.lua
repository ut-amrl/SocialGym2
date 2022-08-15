
function Vector2(x, y)
    return {x = x, y = y}
  end

  function Vector3(x, y, z)
    return {x = x, y = y, z = z}
  end

  function DegToRad(d)
    return math.pi * d / 180
  end
map_name =  "maps/closed/door/t1/closed/door/t1.vectormap.txt"-- Simulator starting location.
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
  