# Simple Gym
A branch for a simplified version of the social gym that strips out the components for pedestrian simulation, PIPS, and training and evaluating models.

## Building and Running Locally

### Dependencies
1. The majority of SocialGym's dependencies can be installed with the included script `scripts/install_dependencies.sh`
2. Refer to `scripts/install_dependencies.sh` for the list of these dependencies.

### Install Instructions
1. Install Dependencies
2. `git clone --recurse_submodule git@github.com:ut-amrl/social_gym.git`
3. Set the necessary path variables for ROS with `scripts/set_paths.sh` (This will need to be done in each terminal for running SocialGym as well)
4. `make`

## Running the Gym
An example script for running the same scenario on repeat is available at
1. `python src/run_gym.py`
2. The environment run by this script is contained in `config/gym_gen/`
3. `sim_config.lua` contains the configuration of the environment, the robots, and includes other configuration files.
4. `humans.lua` is the configuration file for the currently static humans.
5. `launch.launch` starts all of the necessary components when called. It also passes the necessary map to navigation. 
6. The maps themselves are found in `submodules/amrl_maps/`

## Configuring the Benchmark
A number of configuration files configure the behavior and performance of the generated scenarios. If you are interested in configuring SocialGym for a new benchmark, or customizing the existing benchmark in your way. You should familiarize yourself with the following files.
1. The templates for the scenario generation are provided in `src/templates` these templates include the launch files, and various configuration files for the scene. We provide two sets of templates, one for the training map, and another for the transfer task from the SocialGym paper.
  
   `*_launch.launch`: launches all necessary nodes with correct paths to configuration files. Notably, the map used by navigation is configured here.
   
   `sim_config.lua`: Configuration for the environment simulator.
2. `config/ut_jackal_config.lua`: Describes the properties of the robot as simulated by the environment. Included is the default configuration used for all experiments.
3. In addition, the individual submodules used for human simulation and navigation contain additional configuration parameters that can be used to further customize those underlying behaviors. This can be taken as far as using different submodules for these tasks, assuming the alternatives implement the necessary service definitions for communicating with the environment and gym modules.
   

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
