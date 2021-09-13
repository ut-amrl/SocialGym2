sudo apt-get update
sudo apt-get install -y ros-noetic-tf \
  ros-noetic-geometry* libgflags-dev libgoogle-glog-dev lua5.1-dev \
  python2.7 python-is-python3
pip3 install click jinja2 pandas stable-baselines3[extra] gym imitation
