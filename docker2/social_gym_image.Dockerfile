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
RUN pip3 install stable-baselines3[extra]
RUN pip3 install pettingzoo==1.19.0
RUN pip3 install supersuit==3.3.3
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
ARG L_PREV_USR=rosdev
# TODO: You might want to use a strong password here
ARG L_PREV_PWD=12345
ARG ROS_WS=/home/${L_PREV_USR}/social_gym

# Adding a low-privilege user for CLion to SSH
RUN groupadd -r ${L_PREV_USR}
RUN useradd -ms /bin/bash -r -g ${L_PREV_USR} ${L_PREV_USR} && yes ${L_PREV_PWD} | passwd ${L_PREV_USR}
RUN mkdir -p ${ROS_WS} && chown -R ${L_PREV_USR} /home/${L_PREV_USR}
RUN usermod -aG sudo ${L_PREV_USR}

COPY docker2/set_paths.sh /home/rosdev/set_paths.sh


WORKDIR ${ROS_WS}/submodules

SHELL ["/bin/bash", "-c"]

COPY ./submodules/pedsim_ros ./pedsim_ros

RUN apt-get install ros-noetic-roslint
RUN source /home/rosdev/set_paths.sh && cd pedsim_ros && catkin_make

WORKDIR ${ROS_WS}/submodules

COPY ./submodules/amrl_msgs ./amrl_msgs
RUN source /home/rosdev/set_paths.sh && cd amrl_msgs && make

COPY ./submodules/graph_navigation ./graph_navigation
RUN source /home/rosdev/set_paths.sh && cd graph_navigation && make

RUN source /home/rosdev/set_paths.sh && rosdep update

WORKDIR ${ROS_WS}/..
COPY ./docker2/entrypoint.sh ./entrypoint.sh
COPY ./docker2/get_vars.sh ./get_vars.sh