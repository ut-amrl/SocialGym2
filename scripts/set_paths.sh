source /opt/ros/noetic/setup.bash
source submodules/pedsim_ros/devel/setup.bash
DIR=$(pwd)
echo $DIR
export ROS_PACKAGE_PATH=${DIR}/submodules/amrl_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/amrl_maps:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/pips:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/graph_navigation:$ROS_PACKAGE_PATH
# export ROS_PACKAGE_PATH=/submodules/cobot/cobot_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim/src/state_switcher_rviz_plugin:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim/pedestrian_simulation/:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim/src/ros_social_gym:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/:$ROS_PACKAGE_PATH
