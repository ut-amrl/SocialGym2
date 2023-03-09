 <!-- <h1 style="text-align: center;"> SocialGym 2 </h1> -->

<p align="center">
  <img src="https://drive.google.com/uc?id=1E0DkXdHRfS9gjSX33-NaKLS9duwHwK68" width="400" />
</p>


This is the codebase for our multi-agent simulator for real world social navigation. Find the documentation (currently under active development) at https://amrl.cs.utexas.edu/social_gym/index.html.

<!-- 
<p align="center">
  <img src="https://obj.umiacs.umd.edu/badue-accepted/sim_demo.gif" width="400" />
</p> -->



# Installation

**Note**: We require you have installed Docker and Python 3.8 (version 3.8 may not be necessary but it's what we use)

**WARNING**: The authors of this project have had numerous issues with Docker and Docker-Compose when installed via snap.  We
recommend uninstalling docker and docker-compose if you have GPU-related issues with the docker images and installing them via
`apt`.

---

### 1.) Clone the repo and checkout main

```shell
git clone git@github.com:ut-amrl/social_gym.git
git checkout main
```

Although we use Docker to run Social Gym's internals, we have scripts to set up Social Gym as well as scripts that kick
off the process written in python (so you still need to install stuff)

### 2.) Install requirements and run the install script!

```shell
python3 ./scripts/install_config_runner.py
```

Expect this to take a while (20-40m depending on your machine, mostly for the last step.). 

### 3.) Run the Config Runner!

```shell
python3 config_runner/run.py -c 1_31_23/door/sacadrl.json
```

This should open an RVIS window (looks like a 2D grid with options on the side-panels). If you do NOT see windows, there's a problem with the docker file. First try running
```shell
sudo xhost +
```
and repeat the command.  If it still fails to show windows then try adding `--network host`
to the file `{PROJECT_ROOT}/config_runner/run.sh` on lines where we are setting docker environment variables. Otherwise, look up stuff like "cannot display window from docker container" -- this is critical for Social Gym to work.

## If you use this code, please cite the following
---

```
@software{SocialGym2,
author = {Sprague, Zayne and Chandra, Rohan and Holtz, Jarrett and Biswas, Joydeep},
title = {{SocialGym2.0: Simulator for Multi-Agent Social Robot Navigation in Shared Human Spaces}},
url = {https://github.com/ut-amrl/social_gym},
version = {2.0},
doi = {10.1109/IROS47612.2022.9982021}
}
```

## License
---
This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
