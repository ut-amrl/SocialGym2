-- require("config.sim_config");
function Vector2(x, y)
  return {x = x, y = y}
end

function Vector3(x, y, z)
  return {x = x, y = y, z = z}
end

function DegToRad(d)
  return math.pi * d / 180
end

-- MODEL PARAMETERS
invert_linear_vel_cmds = false
invert_angular_vel_cmds = false
linear_pos_accel_limit = 6.0
linear_neg_accel_limit = 6.0
angular_pos_accel_limit = 6.0
angular_neg_accel_limit = 6.0
max_angular = 3.0
max_linear_vel = 10.0
drive_callback_topic = "/navigation/cmd_vel"
-- drive_callback_topic = "/pedbot/control/cmd_vel"
diff_drive_odom_topic = "/jackal_velocity_controller/odom"
linear_odom_scale = 1.0
angular_odom_scale = 1.0


-- SIMULATOR PARAMETERS

-- tf
publish_map_to_odom = true
publish_foot_to_base = true

-- Kinematic
rear_axle_offset = 0.0
min_turn_radius = 0.0

-- laser_topic = "velodyne_2dscan"
laser_loc = Vector3(0.07, 0, 0.5)
car_width = 0.43
car_length = 0.5
car_height = 0.65;

laser_angle_min = DegToRad(-180.0);
laser_angle_max = DegToRad(180.0);
laser_angle_increment = DegToRad(0.1);
laser_min_range = 0.02;
laser_max_range = 150.0;
