# How does SocialGym 2 work ? 

![Overview](https://drive.google.com/uc?id=1TYFzAPYmlWLIgG1PnnJHpauWWh6Npdnn)

At the top of the stack is the [**PettingZoo**](https://github.com/Farama-Foundation/PettingZoo) and [**Stable Baselines3**](https://stable-baselines3.readthedocs.io/en/master/) interface. This interface uses ROS to send actions from a policy to [**UTMRS**](https://github.com/ut-amrl/ut_multirobot_sim), a lightweight simulation engine that acts as an intermediate between the interface and the local navigation and the human crowd simulation modules. The local navigation planner is responsible converting high-level actions from the PettingZoo interface into continuous motion commands that satisfy the underlying robot dynamics and sends back the next state to the simulation engine. Each layer of the stack has a modular API that allows researchers and developers to focus on a single part of the stack at a time without having to refactor or access other parts of the stack.

### Read the paper

[**SOCIALGYM 2.0: Simulator for Multi-Agent Social Robot Navigation in Shared Human Spaces**](https://arxiv.org/pdf/2303.05584.pdf) 

### Local Navigation
The local navigation algorithm in SocialGym 2.0 consists of a way-point follower and a dynamic window-based controller: We first compute a local target using the pure pursuit algorithm. We then use a dynamic window-like approach to generate a set of circular arcs from the current pose of the robot with different curvatures. We determine the optimal arc by minimizing a cost function that is a weighted linear sum of different features such as distance to goal, clearance from obstacles etc. followed by computing the optimal linear and angular velocities that steer the robot along that optimal trajectory. The linear velocity controller chooses between cruising, accelerating, and braking based on the current speed, kinodynamic parameters, and distance to the goal to achieve the optimal velocities for the robot.


### Improvements from SocialGym 1.0
Although a lot has changed in Social Gym 2.0, most of the functionality from Social Gym 1.0 remained with small tweaks required.

- Multi Agent training!
  - Single Agent training is as easy as setting the max number of agents to 1.
  - You can control the observations and rewards of each agent in a modular way. 
  - Vary the number of agents during training and evaluation. 
- Control over the environment and simulator! 
  - Use or create your own Stable Baselines / Pettingzoo environment wrappers to control the environment.
  - Agents can be colored or labeled for visualization needs. 
  - Change scenes/environments as needed throughout training and evaluation. 
  - Toggle partial or full observability. 
- Helper classes to create your own observations and rewards. 
- Easy docker container for creating your own Environment and Navigation Graphs. 
- Tensorboard implementation to help visualize the training process including tracking of important values and recordings of the simulator. 

A permanent link to Social Gym 1.0 can be found [here](https://github.com/ut-amrl/social_gym/tree/release/social_gym_1.0)

The paper for Social Gym 1.0 can also be found [here](https://arxiv.org/abs/2109.11011)

---


