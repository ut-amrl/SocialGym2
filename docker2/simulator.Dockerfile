FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_WS=/catkin_ws

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update \
  && apt-get install -y \
      build-essential \
      psmisc \
      vim-gtk \
  && rm -rf /var/lib/apt/lists/*

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
COPY docker2/ros_global.sh /etc/profile.d/ros_global.sh

RUN apt-get update \
  && apt-get install -y \
      ros-noetic-hector-gazebo-plugins \
  && rm -rf /var/lib/apt/lists/*
RUN apt-get install -y ros-noetic-rviz

CMD source /home/rosdev/social_gym/docker2/set_paths.sh && rosrun rviz rviz -d /home/rosdev/social_gym/submodules/ut_multirobot_sim/visualization.rviz
