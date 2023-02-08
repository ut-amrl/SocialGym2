Welcome to the Social Gym 2.0 documentation!
======================================

(Find the GitHub link at https://github.com/ut-amrl/social_gym)


.. image:: https://obj.umiacs.umd.edu/badue-accepted/sim_demo.gif
  :width: 400
  :align: center

|

(This documentation is currently under active development.)


We present SocialGym 2.0, a multi-agent navigation simulator for social robot research. Our simulator models multiple autonomous agents, replicating real-world dynamics in complex environments, including doorways, hallways, intersections, and roundabouts. Unlike traditional simulators that concentrate on single robots with basic kinematic constraints in open spaces, SocialGym 2.0 employs multi-agent reinforcement learning (MARL) to develop optimal navigation policies for multiple robots with diverse, dynamic constraints in complex environments. Built on the PettingZoo MARL library and Stable Baselines3 API, SocialGym 2.0 offers an accessible python interface that integrates with a navigation stack through ROS messaging. SocialGym 2.0 can be easily installed and is packaged in a docker container, and it provides the capability to swap and evaluate different MARL algorithms, as well as customize observation and reward functions. We also provide scripts to allow users to create their own environments and have conducted benchmarks using various social navigation algorithms, reporting a broad range of social navigation metrics.


.. toctree::
   :maxdepth: 2

   installation/installation
   installation/advanced_usage
   intro/what_is_social_gym
   intro/features
   intro/making_a_new_scene
   contributing/making_docs
   :caption: Contents:


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

