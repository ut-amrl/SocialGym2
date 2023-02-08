#### Training and Evaluation with ConfigRunner

To ease batch training jobs we created a wrapper class around the training and evaluation code that can be configured 
via a configuration file (.yaml file)

These configuration files are stored in `{PROJECT ROOT}/config_runner/configs`

ConfigRunner allows you train and evaluate a policy using one of these files via

```shell
python config_runner/run.py -c {path_to_config}
```

where the `{path_to_config}` is the relative path from `{PROJECT ROOT}/config_runner/configs` to a specific config file.

You can run batch training jobs by either separating each unique configuration file with a white space and the `-c` flag, or you can 
use the `-f` flag (meaning `folder`) and point it at a folder in the `{PROJECT ROOT}/config_runner/configs` directory.

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

### ConfigRunner Configurations

Each attribute in the yaml configuration matches an argument passed into the `run` function in `{PROJECT_ROOT}/src/config_run.py`

They are described in details in the file `{PROJECT ROOT}/src/config_run.py`


