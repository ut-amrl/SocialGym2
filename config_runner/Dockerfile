FROM ros:noetic

RUN apt-get update \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      cmake \
      rsync \
      tar \
      python \
      wget \
  && rm -rf /var/lib/apt/lists/*

# Install Necessary Programs
RUN apt-get update && apt-get install -y git openssh-server ros-noetic-tf \
    ros-noetic-geometry* libgflags-dev libgoogle-glog-dev lua5.1-dev \
    ros-noetic-gazebo-ros ros-noetic-rviz libncurses5-dev python3-pip \
    python2.7 python-is-python3 neovim curl

RUN pip3 install click
RUN pip3 install jinja2
RUN pip3 install pandas
RUN pip3 install supersuit==3.5.0
RUN pip3 install stable-baselines3[extra]
RUN pip3 install pettingzoo==1.19.0
RUN pip3 install gym==0.21.0
RUN pip3 install imitation

# Download and install Z3
WORKDIR /tmp
RUN apt-get install -y unzip wget
RUN wget https://github.com/Z3Prover/z3/releases/download/z3-4.8.9/z3-4.8.9-x64-ubuntu-16.04.zip
RUN unzip z3-4.8.9-x64-ubuntu-16.04
RUN cp z3-4.8.9-x64-ubuntu-16.04/bin/libz3.* /usr/bin
RUN cp z3-4.8.9-x64-ubuntu-16.04/include/* /usr/include



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

RUN pip3 install tensorboardX

RUN apt-get update --fix-missing
RUN apt-get install -y ros-noetic-cv-bridge
RUN pip3 install opencv-python-headless==4.5.2.52
RUN pip3 install moviepy

RUN apt-get -y install gdb

ENV ROS_ROOT=/opt/ros/noetic/share/ros

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

RUN apt-get install -y libtf2-ros-dev

RUN apt install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_WS=/home/rosdev/social_gym

WORKDIR ${ROS_WS}

SHELL ["/bin/bash", "-c"]

#COPY ./submodules/pedsim_ros ./pedsim_ros
#
RUN apt-get install ros-noetic-roslint

RUN pip install sb3_contrib optuna

COPY ./submodules ${ROS_WS}/submodules


WORKDIR ${ROS_WS}/submodules
COPY ./config_runner/set_paths__pre_pedsim.sh ${ROS_WS}/config_runner/set_paths__pre_pedsim.sh
COPY ./config_runner/set_paths.sh ${ROS_WS}/config_runner/set_paths.sh

RUN source ${ROS_WS}/config_runner/set_paths__pre_pedsim.sh && cd pedsim_ros && catkin_make


COPY ./submodules/amrl_msgs ./amrl_msgs
RUN source ${ROS_WS}/config_runner/set_paths.sh && cd amrl_msgs && make

COPY ./submodules/graph_navigation ./graph_navigation
RUN source ${ROS_WS}/config_runner/set_paths.sh && cd graph_navigation && make

COPY ./submodules/ut_multirobot_sim ./ut_multirobot_sim
RUN source ${ROS_WS}/config_runner/set_paths.sh && rosdep update

RUN source ${ROS_WS}/config_runner/set_paths.sh && cd ut_multirobot_sim && make

RUN source ${ROS_WS}/config_runner/set_paths.sh && rosdep update
RUN pip3 install stable-baselines3[extra]==1.6.2
RUN pip3 install sb3_contrib
RUN pip3 install torch==1.12.1
COPY ./scripts ${ROS_WS}/scripts

COPY ./config ${ROS_WS}/config
COPY ./manifest.xml ${ROS_WS}/manifest.xml
COPY ./config_runner ${ROS_WS}/config_runner

COPY ./src ${ROS_WS}/src

WORKDIR ${ROS_WS}

RUN pip3 install supersuit==3.5.0
RUN pip3 install gym==0.26.2


#RUN source scripts/set_paths.sh && make
