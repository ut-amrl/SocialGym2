#!/bin/bash

ros_env="ROS_ROOT ROS_MASTER_URI ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH PATH ROS_DISTRO ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION"
env_string=""
for e in ${ros_env}; do
    env_string+="$e=${!e};"
done
echo "$env_string"