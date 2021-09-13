# social_gym
SocialGym: A benchmarking framework for social robot navigation.

## Related Publications

## Building and Running with Docker

## Building and Running Locally

### Dependencies
1. The majority of SocialGym's dependencies can be installed with the included script `scripts/install_dependencies.sh`
2. Refer to `scripts/install_dependencies.sh` for the list of these dependencies.
3. [Z3](https://github.com/Z3Prover/z3/releases) (version 4.8.9)

### Install Instructions
1. Install Dependencies
2. `git clone --recurse_submodule git@github.com:ut-amrl/social_gym.git`
3. Set the necessary path variables for ROS with `scripts/set_path.sh` (This will need to be done in each terminal for running SocialGym as well)
4. `make -j N` (N is the number of cores you want to use.)

## Recreating our Experiments

### Training and Evaluating Approaches
1. Training DQN and PPO can be done with their respective train scripts either locally or in the docker image.
   `<scripts/<local|docker>\train_dqn.py`
   `<scripts/<local|docker>\train_ppo.py`
2. Training the LfD algorithms requires downloading the training data, provided here, then running the provided scripts
   `<scripts/<local|docker>\train_bc.py`
   `<scripts/<local|docker>\train_gail.py`
   `<scripts/<local|docker>\train_pips.py`
3. To evaluate the trained models:   
   `<scripts/<local|docker>\eval_ga.py`
   `<scripts/<local|docker>\eval_ref.py`
   `<scripts/<local|docker>\eval_dqn.py`
   `<scripts/<local|docker>\eval_ppo.py`
   `<scripts/<local|docker>\eval_bc.py`
   `<scripts/<local|docker>\eval_gail.py`
   `<scripts/<local|docker>\eval_pips.py`
4. The data from evaluation will be output to `./data/`
5. You can visualize the results by first preprocessing the data with `preprocess.py`, and then visualize with the provided juypter notebooks
   `cd notebooks && juypter notebook`

## Using our Benchmark + Your Approach
1. Create a train/eval script matching the setup of our experiments, with your approach instead of the reference approaches. Refer to the ppo examples for reference.
2. Run training, then evaluation with your model of choice from the training process.
3. The data from evaluation will be output to `./data`
4. You can visualize your results with the provided juypter notebooks by preprocessing the data and modifying the scripts to point to your new results.

## Customizing or Creating Benchmarks
Coming Soon!

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
