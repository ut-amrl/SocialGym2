Welcome to the Social Gym 2.0 documentation!
======================================

.. (Find the GitHub link at https://github.com/ut-amrl/social_gym https://obj.umiacs.umd.edu/badue-accepted/sim_demo.gif)


.. image:: https://drive.google.com/uc?id=1-mdW21SIJiF4LUlxGxQClDlZhd5iHUDP
  :width: 400
  :align: center

|

.. (This documentation is currently under active development.)


SocialGym 2.0 is a lightweight 2D simulation environment for robot social navigation.  It allows for multi-agent and single-agent scenarios as well as dynamic obstacles for testing complex social navigation.  Social Gym's foundation is in ROS (Robot Operating System) which we surface via a python API.  This allows for all the training, evaluation, and experimentation to be done in Python similar to Stable Baselines or PettingZoo.  However, because it's built on top of ROS, it's also easy to extend with ROS submodules to experiment with more complex simulations like lidar maps etc. 

It's important to note that you do not need to know ROS or C++ to develop and work with Social Gym.  We worked hard to make the implementation abstracted from each other! 

.. toctree::
   :maxdepth: 2

   installation/installation
   installation/advanced_usage
   intro/features
   intro/making_a_new_scene
   intro/training_and_eval
   contributing/making_docs
   :caption: Contents:


If you use this code, please cite the following
==================

.. code-block:: latex

  @article{sprague2023socialgym,
  title={SOCIALGYM 2.0: Simulator for Multi-Agent Social Robot Navigation in Shared Human Spaces},
  author={Sprague, Zayne and Chandra, Rohan and Holtz, Jarrett and Biswas, Joydeep},
  journal={arXiv preprint arXiv:2303.05584},
  year={2023}}

.. code-block:: latex

  @inproceedings{holtz2022socialgym,
    title={Socialgym: A framework for benchmarking social robot navigation},
    author={Holtz, Jarrett and Biswas, Joydeep},
    booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    pages={11246--11252},
    year={2022},
    organization={IEEE}
  }

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

