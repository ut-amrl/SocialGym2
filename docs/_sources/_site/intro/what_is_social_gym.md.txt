# What is Social Gym

Social Gym is a lightweight 2D simulation environment for robot social navigation.  It allows for multi-agent and
single-agent scenarios as well as dynamic obstacles for testing complex social navigation.  

### Social Gym's Stack

Social Gym's foundation is in ROS (Robot Operating System) which we surface via a python API.  This allows for all the 
training, evaluation, and experimentation to be done in Python similar to Stable Baselines or PettingZoo.  However, 
because it's built ontop of ROS, it's also easy to extend with ROS submodules to experiment with more complex 
simulations like lidar maps etc. 

> It's important to note that you do not need to know ROS or C++ to develop and work with Social Gym.  We work hard to 
> make the implementation abstracted from each other!


