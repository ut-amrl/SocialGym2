# Training and evaluation a policy

Training a policy using multi-agent reinforcement learning is as simple as editing a config file.

### Edit the config file to be passed to SocialGym

To ease batch training jobs we created a wrapper class around the training and evaluation code that can be configured  via a configuration file (.yaml file). These configuration files are stored in `{PROJECT ROOT}/config_runner/configs`. ConfigRunner allows you train and evaluate a policy using one of these files via

```shell
python config_runner/run.py -c {path_to_config}
```

where the `{path_to_config}` is the relative path from `{PROJECT ROOT}/config_runner/configs` to a specific config file. You can run batch training jobs by either separating each unique configuration file with a white space and the `-c` flag, or you can use the `-f` flag (meaning `folder`) and point it at a folder in the `{PROJECT ROOT}/config_runner/configs` directory. 

An example of a config is shown below:

```yaml
{
  "num_agents": [[0, 3], [35, 4], [70, 5]],
  "eval_num_agents": [3, 4, 5, 7, 10],
  "train_length": 250000,
  "ending_eval_trials": 25,
  "eval_frequency": 0,
  "intermediate_eval_trials": 25,
  "policy_algo_sb3_contrib": false,
  "policy_algo_name": "PPO",
  "policy_name": "MlpPolicy",
  "policy_algo_kwargs": {"n_steps":  4096},
  "monitor": false,

  "experiment_names": ["envs_door"],

  "run_name": "door_ao",
  "run_type": "AO",
  "device": "cuda:0",

  "other_velocities_obs": true,
  "agent_velocity_obs": true,

  "agent_velocity_ignore_theta": false,
  "other_velocities_ignore_theta": false,
  "other_poses_ignore_theta": false,
  "agent_pose_ignore_theta": false,

  "entropy_constant_penalty": -100000,
  "entropy_constant_penalty_only_those_that_did_not_finish": true,

  "timelimit": true,
  "timelimit_threshold": 3000
}
```

### The main training loop using the config file you just edited

Each attribute in the yaml configuration matches an argument passed into the `run` function in `{PROJECT_ROOT}/src/config_run.py`. Social Gym 2.0 uses a familiar Gym-like training loop with some important deviations, an example of training a new policy using a config file is shown below.

```python
# 1.) Create the Scenario
scenario = GraphNavscenario('envs/scenario/hallway')

# 2.) Creating the Observer through modular Observations that are customizable
observations = [
  AgentsPose(ignore_theta=True), 
  OtherAgentObservables(ignore_theta=True),
  CollisionObservation(),
  SuccessObservation()
]
observer = Observer(observations)

# 3.) Creating the Rewarder with a sparse goal reward and a penalty term that scales over the course of training.
rewards = [
  Success(weight=100),
  LinearWeightScheduler(Collisions(), duration=10_000)
]
rewarder = Rewarder(rewards)

# 4.) Create the base class
env = RosSocialEnv(observer, rewarder, scenario, num_agents=7)

# 5.) Custom wrappers
env = EntropyEpisodeEnder(env)
env = NewScenarioWrapper(env, new_scenario_episode_frequency=1, plans=num_agents if isinstance(num_agents, list) else [0, num_agents])

# 6.) Wrappers that convert PettingZoo into a Stable Baselines v3 environment
env = ss.black_death_v3(env)
env = ss.pad_observations_v0(env)
env = ss.pad_action_space_v0(env)
env = ss.pettingzoo_env_to_vec_env_v1(env)
env.black_death = True

env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')

# 7.) Stable Baselines v3 normalization and monitoring wrappers.
env = VecNormalize(env, norm_reward=True, norm_obs=True, clip_obs=10.)
env = VecMonitor(env)

  
# 8.) Standard Gym Interfacing for Training and Stepping 
model = PPO("MlpPolicy", env)
model.learn(total_timesteps=10_000)

# 9.) Stepping through the environment with a trained policy.
obs = env.reset()
while env.agents:
    action, _states = model.predict(obs)
    obs, rewards, terminations, infos = env.step(actions)
```



