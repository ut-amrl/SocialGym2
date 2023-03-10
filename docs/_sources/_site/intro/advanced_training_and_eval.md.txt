# Advanced Training and Evaluation

Social Gym 2.0 uses a familiar Gym-like training loop with some important deviations, an example of training a new 
policy is shown below.

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

