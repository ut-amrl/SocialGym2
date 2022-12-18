source /opt/ros/noetic/setup.bash
#source /home/rosdev/social_gym/submodules/pedsim_ros/devel/setup.bash

DIR=$(pwd)
echo $DIR
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/amrl_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/amrl_maps:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/ut_multirobot_sim:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/pips:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/graph_navigation:$ROS_PACKAGE_PATH

# export ROS_PACKAGE_PATH=/submodules/cobot/cobot_msgs:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/ut_multirobot_sim/src/state_switcher_rviz_plugin:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/ut_multirobot_sim/pedestrian_simulation/:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/submodules/ut_multirobot_sim/src/ros_social_gym:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rosdev/social_gym/:$ROS_PACKAGE_PATH

export PYTHONPATH=$PYTHONPATH:/home/rosdev/social_gym
export ROS_MASTER_URI=http://rosmaster:11311