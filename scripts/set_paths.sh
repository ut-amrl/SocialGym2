source /opt/ros/noetic/setup.bash
DIR=$(pwd)
echo $DIR
export ROS_PACKAGE_PATH=${DIR}/submodules/amrl_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/amrl_maps:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/graph_navigation:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim/src/state_switcher_rviz_plugin:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/src:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/submodules/ut_multirobot_sim:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=${DIR}/:$ROS_PACKAGE_PATH
