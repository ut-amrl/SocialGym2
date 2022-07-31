# Base Image: Ros Noetic
FROM ros:noetic
SHELL ["/bin/bash", "-c"]

# Set the working director to root
WORKDIR /root


# Install Necessary Programs
RUN apt-get update && apt-get install -y git openssh-server ros-noetic-tf \
    ros-noetic-geometry* libgflags-dev libgoogle-glog-dev lua5.1-dev \
    ros-noetic-gazebo-ros ros-noetic-rviz libncurses5-dev python3-pip \
    python2.7 python-is-python3 neovim curl

RUN #apt install software-properties-common -y
RUN #add-apt-repository ppa:deadsnakes/ppa -y

#RUN apt-get update && apt-get install -y python3.7 python3.7-distutils

RUN #update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
RUN #curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN #python3.7 get-pip.py --force-reinstall

RUN #pip3 install gym_collision_avoidance


#RUN \
#   echo 'alias python3="/usr/bin/python3.6"' >> /root/.bashrc && \
#   echo 'alias pip3="/usr/bin/pip3.7"' >> /root/.bashrc && \
#   source /root/.bashrc

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

RUN cp -r /root/social_gym /project

WORKDIR /root/social_gym/

# To work off the simple-sgym branch we have to move all the folders we compiled into a tmp folder then bring them
# back to avoid git from accidentally wiping the submodule folders back to their uninstalled/compiled states.
# Note: This is only necessary if you are working off anything other than the master branch
RUN chmod 777 /root/social_gym
RUN cp -r /root/social_gym/submodules /tmp
RUN #git checkout simple-sgym
RUN cp -r /tmp/submodules /root/social_gym/


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

RUN #update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1
RUN #pip3 uninstall -y netifaces
RUN #pip3 install netifaces


# We need to source the set_paths then start the roscore program in the background.  After than we can move into the
# main project folder and call our python scripts.
ENTRYPOINT ["/bin/bash", "-c", "cd /root/social_gym && source ./scripts/set_paths.sh && (roscore &) && \"$@\"", "-s"]

# Docker container should be run with these params
# --name SOCIAL_GYM -d -t -i -e DISPLAY=unix:0 -e QT_X11_NO_MITSHM=1 -w /root/social_gym/ -v /home/zayne/Research/RL/social_gym:/opt/project -v /home/zayne/Research/RL/social_gym/config:/root/social_gym/config -v /home/zayne/Research/RL/social_gym/submodules/ut_multirobot_sim/maps:/root/social_gym/submodules/ut_multirobot_sim/maps -v /home/zayne/Research/RL/social_gym/scripts:/root/social_gym/scripts -v /home/zayne/Research/RL/social_gym/src:/home/zayne/Research/RL/social_gym/src -v /home/zayne/.Xauthority:/root/.Xauthority:ro --net host --rm --ipc host
# -d -t -i -e DISPLAY=unix:0 -e QT_X11_NO_MITSHM=1 -w /root/social_gym/ -v /home/zayne/Research/RL/social_gym/scripts:/root/social_gym/scripts -v /home/zayne/Research/RL/social_gym/src:/root/social_gym/src -v /home/zayne/.Xauthority:/root/.Xauthority:ro --net host --rm --ipc host