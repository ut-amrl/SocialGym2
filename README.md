 <!-- <h1 style="text-align: center;"> SocialGym 2 </h1> -->

<p align="center">
  <img src="https://drive.google.com/uc?id=1y3vYhN6z55B0k7SRHf7gycan3rzWSsj8" width="400" />
</p>

This is the codebase for our multi-agent simulator for real world social navigation. Installation instructions ca be found below. Tutorials are provided in the [**documentation**](https://amrl.cs.utexas.edu/SocialGym2/index.html) (currently under active development).

<!-- 
<p align="center">
  <img src="https://obj.umiacs.umd.edu/badue-accepted/sim_demo.gif" width="400" />
</p> -->
# Read the paper 
 [**SOCIALGYM 2.0: Simulator for Multi-Agent Social Robot Navigation in Shared Human Spaces**](https://arxiv.org/pdf/2303.05584.pdf), AAAI 2024.

# Installation

### Prerequisites

You need to have installed Docker and have a GPU.

<!---, and have installed the [**NVIDIA-container runtime**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).--->

**WARNING**: The authors of this project have had numerous issues with Docker and Docker-Compose when installed via snap.  We recommend uninstalling docker and docker-compose if you have GPU-related issues with the docker images and installing them via `apt`.

---

### 1.) Clone the repo and checkout main

```shell
git clone git@github.com:ut-amrl/SocialGym2.git
git checkout main
```

### 2.) Install requirements and run the install script

```shell
python3 ./scripts/install_config_runner.py
```

### 3.) Run the Config Runner!

```shell
python3 config_runner/run.py -c 1_31_23/door/sacadrl.json
```

This should open an RVIS window (looks like a 2D grid with options on the side-panels). If you do NOT see this window, there's a problem with the docker file. First try running
```shell
sudo xhost +
```
and repeat the command.  If it still fails to show windows then try adding `--network host`
to the file `{PROJECT_ROOT}/config_runner/run.sh` on lines where we are setting docker environment variables. Otherwise, look up stuff like "cannot display window from docker container" -- this is critical for Social Gym to work.

### 3.) Have fun!
<p align="center">
  <img src="https://drive.google.com/uc?id=1-mdW21SIJiF4LUlxGxQClDlZhd5iHUDP" />
</p>

## If you use this code, please cite the following
---

```
@inproceedings{chandra2024socialgym,
  title={SOCIALGYM 2.0: Simulator for Multi-Robot Learning and Navigation in Shared Human Spaces},
  author={Chandra, Rohan and Sprague, Zayne and Biswas, Joydeep},
  booktitle={Proceedings of the AAAI Conference on Artificial Intelligence},
  volume={38},
  number={21},
  pages={23778--23780},
  year={2024}
}
```

```
@inproceedings{holtz2022socialgym,
  title={Socialgym: A framework for benchmarking social robot navigation},
  author={Holtz, Jarrett and Biswas, Joydeep},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={11246--11252},
  year={2022},
  organization={IEEE}
}
```

<!-- ```
@software{SocialGym2,
author = {Sprague, Zayne and Chandra, Rohan and Holtz, Jarrett and Biswas, Joydeep},
title = {{SocialGym2.0: Simulator for Multi-Agent Social Robot Navigation in Shared Human Spaces}},
url = {https://github.com/ut-amrl/social_gym},
version = {2.0},
doi = {10.1109/IROS47612.2022.9982021}
}
``` -->

## License
---
This project is released under the MIT License. Please review the [**License file**](LICENSE) for more details.
