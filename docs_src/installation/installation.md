# Installation

In this doc we are going to setup Social Gym's `config runner` which allows you to run Social Gym using a configuration
file.  If all you need to do is train, evaluate, or test your installation of Social Gym, this is the only installation
step you have to follow.  However, if you wish to contribute and develop on Social Gym, we recommend the more indepth
installation guide which goes over setting up a debugger.

### Why are there two installation guides for Social Gym 

Social Gym uses ROS (Robot Operating System) submodules written in C, to ensure they installed correctly and can run on
your machine, we use Docker.  However, this poses challenges for debugging both the Python code and the C code because
everything has to communicate with each other.  To ease the process of setting Social Gym up, we separated installing 
Social Gym for running and experimenting from installing Social Gym for developing.

### Let's Install Config Runner!

---

**Note**: We require you have installed Docker and Python 3.8 (version 3.8 may not be necessary but it's what we use)

**WARNING**: The authors of this project have had numerous issues with Docker and Docker-Compose when installed via snap.  We
recommend uninstalling docker and docker-compose if you have GPU-related issues with the docker images and installing them via
`apt`.

---

#### 1.) Clone the repo and checkout the right branch! (Eventually this will be main)

```shell
git clone git@github.com:ut-amrl/social_gym.git
git checkout main
```

Although we use Docker to run Social Gym's internals, we have scripts to set up Social Gym as well as scripts that kick
off the process written in python (so you still need to install stuff)

#### 2.) Install requirements and run the install script!

```shell
python ./scripts/install_config_runner.py
```

Expect this to take a while (20-40m depending on your machine, mostly for the last step.). 

#### 3.) Run the Config Runner!

```shell
python config_runner/run.py -c 11_20_22/sacadrl.json
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
#### 4.) Have fun!

You can now specify your own configurations and run your own training or evaluation jobs.  More documentation on this
later.

If you need to develop or debug Social Gym -- you'll have to follow the full installation guide on the next
page.