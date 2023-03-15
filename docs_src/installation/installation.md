# Installation

In this doc we are going to setup Social Gym's `config runner` which allows you to run [**Social Gym**](https://github.com/ut-amrl/SocialGym2) using a configuration file. If all you need to do is train, evaluate, or test your installation of Social Gym, this is the only installations tep you have to follow.  However, if you wish to contribute and develop on Social Gym, we recommend the advanced installation which goes over setting up a debugger.

<!-- ### Why are there two installation guides for Social Gym 

Social Gym uses ROS (Robot Operating System) submodules written in C, to ensure they installed correctly and can run on
your machine, we use Docker.  However, this poses challenges for debugging both the Python code and the C code because
everything has to communicate with each other.  To ease the process of setting Social Gym up, we separated installing 
Social Gym for running and experimenting from installing Social Gym for developing. -->

<!-- ### Let's Install Config Runner! -->

---
#### Prerequisites

You need to have installed Docker, have an NVIDIA GPU, and have installed the [**NVIDIA-container runtime**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

**WARNING**: The authors of this project have had numerous issues with Docker and Docker-Compose when installed via snap.  We recommend uninstalling docker and docker-compose if you have GPU-related issues with the docker images and installing them via `apt`.

---

#### Step 1: Clone the repo and checkout main

```shell
git clone git@github.com:ut-amrl/SocialGym2.git
git checkout main
```

Although we use Docker to run Social Gym's internals, we have scripts to set up Social Gym as well as scripts that kick
off the process written in python (so you still need to install stuff)

#### Step 2: Install requirements and run the install script!

```shell
python3 ./scripts/install_config_runner.py
```

Expect this to take a while (20-40m depending on your machine, mostly for the last step.). 

#### Step 3: Run the Config Runner!

```shell
python3 config_runner/run.py -c 1_31_23/door/sacadrl.json
```

This should open an RVIS window (looks like a 2D grid with options on the side-panels).  It may CLOSE and re-open MANY 
times.  This is because of a known bug where ROS hangs on some dependency unnecessarily (we are trying to fix this).  

The way our code gets around it is by restarting the docker container if it fails to start running after so many 
seconds.

---

**NOTE**: If you do NOT see windows, there's a problem with the docker file.

First try running
```shell
sudo xhost +
```
and repeat the command.  If it still fails to show windows then try adding 

`--network host`
to the file `{PROJECT_ROOT}/config_runner/run.sh` on lines where we are setting docker environment variables. 
Otherwise, look up stuff like "cannot display window from docker container" -- this is critical for Social Gym to work.
We will fill out more tips/tricks and common failures here as we experience them.

---
<!-- #### 4.) Have fun!

You can now specify your own configurations and run your own training or evaluation jobs.  More documentation on this
later.

If you need to develop or debug Social Gym -- you'll have to follow the full installation guide on the next
page. -->