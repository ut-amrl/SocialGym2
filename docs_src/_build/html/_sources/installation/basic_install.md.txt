# Basic Install

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

---

#### 1.) Checkout the right branch! (Eventually this will be main)

```shell
git checkout feature/20221218_multi_agent_finishing_and_cleanup
git submodule update --init --recursive
```

Although we use Docker to run Social Gym's internals, we have scripts to set up Social Gym as well as scripts that kick
off the process written in python (so you still need to install stuff)

#### 2.) Build Vector Display (this creates the environments!)

Assuming you are starting from the ROOT_DIRECTORY of the project:
```shell
cd docker/vectordisplay
docker build -t  vector_display:custom .
cd ../..
```

Expect this to take a while (15-30m depending on your machine).

#### 3.) Create a virtualenv to install python dependencies

Assuming linux/mac style, windows have their own special flavor for activating virtualenvs.
```shell
virtualenv venv
source venv/bin/activate
```

#### 4.) Install python requirements

```shell
pip install -r requirements.txt
```

#### 5.) Create the Environment for Social Gym (don't worry, this is copying files really.)

```shell
python scripts/create_env_template.py -n exp1/train/easy
```

The `--name` parameter is referencing a saved file checked in by a commit, you should expect to see two windows when 
running this command.  It is sufficient to just hit `[ESC]` when they appear.  However, they should show blue lines on 
the first window that look like two rooms separated by a narrow hallway.  The second window should show the same blue 
lines but now inside the two rooms are pink lines (these are navigation paths).

---

**NOTE**: If you do NOT see windows, there's a problem with the docker file.

First try running
```shell
sudo xhost +
```
and repeat the command.  If it still fails to show windows then try adding 

`--network host`
to the file `{PROJECT_ROOT}/docker/vectordisplay/vd.sh` on lines where we are setting docker environment variables. 
Otherwise, look up stuff like "cannot display window from docker container" -- this is critical for Social Gym to work.
We will fill out more tips/tricks and common failures here as we experience them.

---

#### 6.) Run the Config Runner!

```shell
python config_runner/run.py -c 11_20_22/sacadrl.json
```

This should open an RVIS window (looks like a 2D grid with options on the side-panels).  It may CLOSE and re-open MANY 
times.  This is because of a known bug where ROS hangs on some dependency unnecessarily (we are trying to fix this).  

The way our code gets around it is by restarting the docker container if it fails to start running after so many 
seconds.

---

**NOTE**: If you do not see a window appear, follow the same tips from step 5 but for the file 
`config_runner/run_config.sh`.

---

#### 7.) Have fun!

You can now specify your own configurations and run your own training or evaluation jobs.  More documentation on this
later.

If you need to develop or debug Social Gym -- you'll have to follow the full installation guide on the next
page.