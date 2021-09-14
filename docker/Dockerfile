# Base Image: Ros Noetic
FROM ros:noetic
SHELL ["/bin/bash", "-c"]

# Set the working director to root
WORKDIR /root

# Install Necessary Programs
RUN apt-get update && apt-get install -y git openssh-server ros-noetic-tf \
    ros-noetic-geometry* libgflags-dev libgoogle-glog-dev lua5.1-dev \
    ros-noetic-gazebo-ros ros-noetic-rviz libncurses5-dev python3-pip \
    python2.7 python-is-python3 neovim

RUN pip3 install click
RUN pip3 install jinja2
RUN pip3 install pandas
RUN pip3 install stable-baselines3[extra]
RUN pip3 install gym
RUN pip3 install imitation

# Download and install Z3
WORKDIR /tmp
RUN apt-get install -y unzip wget
RUN wget https://github.com/Z3Prover/z3/releases/download/z3-4.8.9/z3-4.8.9-x64-ubuntu-16.04.zip
RUN unzip z3-4.8.9-x64-ubuntu-16.04
RUN cp z3-4.8.9-x64-ubuntu-16.04/bin/libz3.* /usr/bin
RUN cp z3-4.8.9-x64-ubuntu-16.04/include/* /usr/include

WORKDIR /root

# Clone, Source, and Build the requiste git repos
# Clone and build a single repo
# Messages
RUN git clone https://github.com/ut-amrl/social_gym.git --recurse-submodules

WORKDIR /root/social_gym/submodules/pedsim_ros/

RUN source ../../scripts/set_paths.sh && catkin_make

WORKDIR /root/social_gym/

# Setup Environment
RUN source scripts/set_paths.sh

# Build all Packages
RUN source scripts/set_paths.sh && make
