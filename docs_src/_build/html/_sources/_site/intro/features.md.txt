# What's new in Social Gym 2.0 from the original release?

Social Gym 2.0 focused on multiagent training, enhanced feature extraction and policy modeling, modular control over
environments and training, as well as various quality of life improvements to make training, evaluating, experimenting,
and developing easier!

---

A permanent link to Social Gym 1.0 can be found [here](https://github.com/ut-amrl/social_gym/tree/release/social_gym_1.0)

The paper for Social Gym 1.0 can also be found [here](https://arxiv.org/abs/2109.11011)

---

Although a lot has changed in Social Gym 2.0, most of the functionality from Social Gym 1.0 remained with small tweaks
required. An itemized list of features are below with indicators (*) for new features!

# Social Gym Features

- Multi Agent training! *
  - Single Agent training is as easy as setting the max number of agents to 1.
  - You can control the observations and rewards of each agent in a modular way. *
  - Vary the number of agents during training and evaluation. *
- Control over the environment and simulator! *
  - Use or create your own Stable Baselines / Pettingzoo environment wrappers to control the environment.
  - Agents can be colored or labeled for visualization needs. *
  - Change scenes/environments as needed throughout training and evaluation. *
  - Toggle partial or full observability. *
- Helper classes to create your own observations and rewards. *
- Easy docker container for creating your own Environment and Navigation Graphs. *
- Tensorboard implementation to help visualize the training process including tracking of important values and recordings of the simulator. *