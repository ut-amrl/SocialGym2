# social_gym
SocialGym: A benchmarking framework for social robot navigation.

## Related Publications

## Building and Running Locally

### Dependencies
1. The majority of SocialGym's dependencies can be installed with the included script `scripts/install_dependencies.sh`
2. Refer to `scripts/install_dependencies.sh` for the list of these dependencies.
3. [Z3](https://github.com/Z3Prover/z3/releases) (version 4.8.9)

### Install Instructions
1. Install Dependencies
2. `git clone --recurse_submodule git@github.com:ut-amrl/social_gym.git`
3. Set the necessary path variables for ROS with `scripts/set_paths.sh` (This will need to be done in each terminal for running SocialGym as well)
4. `make -j N` (N is the number of cores you want to use.)

## Recreating our Experiments

### Training and Evaluating Approaches
1. `mkdir data`
2. Training DQN and PPO can be done with their respective train scripts either locally or in the docker image.
   `<src/train_dqn.py`
   `<src/train_ppo.py`
3. Training the LfD algorithms requires downloading the training data, provided here, then running the provided scripts
   `<src/train_bc.py`
   `<src/train_gail.py`
   `<src/train_pips.py`
4. To evaluate the trained models:   
   `<src/eval_ga.py`
   `<src/eval_ref.py`
   `<src/eval_dqn.py`
   `<src/eval_ppo.py`
   `<src/eval_bc.py`
   `<src/eval_gail.py`
   `<src/eval_pips.py`
5. The data from evaluation will be output to `./data/`
6. Preprocess the data with `preprocess_p1-3.py`
7. You can visualize the results by first preprocessing the data with `preprocess.py`, and then visualize with the provided juypter notebooks
   `cd notebooks && juypter notebook`. You will need to customize the jupyter notebooks with the correct paths to files.

## Using our Benchmark + Your Approach
1. Create a train/eval script matching the setup of our experiments, with your approach instead of the reference approaches. Refer to the ppo examples for reference.
2. Run training, then evaluation with your model of choice from the training process.
3. The data from evaluation will be output to `./data`
4. You can visualize your results with the provided juypter notebooks by preprocessing the data and modifying the scripts to point to your new results. You will need to customize the notebooks with the correct policy names and paths to files.

## Building and Running with Docker
To run SocialGym in a Docker environment you can follow these instructions to build the SocialGym Docker images.
1. `git clone --recurse_submodule git@github.com:ut-amrl/social_gym.git`
2. `cd social_gym/docker`
3. `docker build -t social_gym:1.0 .`
4. This will create an image social_gym:1.0 that can be used to run experiments interactively.
5. In addition to using the docker container to interactively setup and run experiments, any of our provided scripts can be spun up in its own container.
6. To do this follow the example shown in `docker/run_training.sh`, which setups up a container and runs the PPO training script in that container.
7. By changing the final command called in the Docker, this can be done for any script in the image.

## Configuring the Benchmark
A number of configuration files configure the behavior and performance of the generated scenarios. If you are interested in configuring SocialGym for a new benchmark, or customizing the existing benchmark in your way. You should familiarize yourself with the following files.
1. The templates for the scenario generation are provided in `src/templates` these templates include the launch files, and various configuration files for the scene. We provide two sets of templates, one for the training map, and another for the transfer task from the SocialGym paper.
   
   `*scene.xml`: controls the scene as understood by the human simulator pedsim_ros. This contains a version of the map that is used by the humans, and some human configuration.
   
   `*_launch.launch`: launches all necessary nodes with correct paths to configuration files. Notably, the map used by navigation is configured here.
   
   `pedsim_launch.lauch`: contains additional pedsim configuration, and launches the pedsim human simulator.
   
   `sim_config.lua`: Configuration for the environment simulator.
2. `config/ut_jackal_config.lua`: Describes the properties of the robot as simulated by the environment. Included is the default configuration used for all experiments.
3. In addition, the individual submodules used for human simulation and navigation contain additional configuration parameters that can be used to further customize those underlying behaviors. This can be taken as far as using different submodules for these tasks, assuming the alternatives implement the necessary service definitions for communicating with the environment and gym modules.
   

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
