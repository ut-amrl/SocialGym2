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
RUN git clone https://github.com/ut-amrl/social_gym.git --recurse-submodules
WORKDIR /root/social_gym/submodules/pedsim_ros/
RUN source ../../scripts/set_paths.sh && catkin_make
WORKDIR /root/social_gym/
RUN source scripts/set_paths.sh
RUN source scripts/set_paths.sh && make

WORKDIR /root
RUN git clone https://github.com/ut-amrl/vector_display.git

WORKDIR /root/vector_display

RUN ls
RUN rm -r maps
RUN git clone https://github.com/ut-amrl/amrl_maps.git ./maps
RUN rm -r src/shared
RUN git clone https://github.com/ut-amrl/amrl_shared_lib.git src/shared
RUN rm -r src/config_reader
RUN git clone https://github.com/ut-amrl/config-reader.git src/config_reader

RUN cd /root/social_gym && source scripts/set_paths.sh && cd /root/vector_display && export ROS_PACKAGE_PATH=/root/vector_display:$ROS_PACKAGE_PATH && make

ENV ROS_PACKAGE_PATH=/root/vector_display

# Make QT connect to the monitor and X11 software (Not all of this may be needed, things were added iteratively until
# it worked.
RUN yes | pip3 uninstall opencv-python opencv_contrib_python
RUN pip3 install opencv_contrib_python==3.4.14.51
RUN pip3 install opencv_python==4.5.5.64

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV QT_QUICK_BACKEND=software
ENV QT_GRAPHICSSYSTEM="native"
ENV QT_QPA_PLATFORM=xcb
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DISPLAY=:0
ENV QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins

RUN apt -y install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev qt5-default libxmlrpcpp-dev

ENTRYPOINT ["/bin/bash", "-c", "cd /root/social_gym && source ./scripts/set_paths.sh && export ROS_PACKAGE_PATH=/root/vector_display:$ROS_PACKAGE_PATH && (roscore &) && cd /root/vector_display &&  \"$@\"", "-s"]

