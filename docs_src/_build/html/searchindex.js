Search.setIndex({docnames:["index","installation/basic_install","installation/full_install","intro/features","intro/making_a_new_scene","intro/what_is_social_gym","modules/modules","modules/src","modules/src.environment","modules/src.environment.callbacks","modules/src.environment.environment_types","modules/src.environment.extractors","modules/src.environment.observations","modules/src.environment.observations.types","modules/src.environment.observations.types.manual_zone","modules/src.environment.rewards","modules/src.environment.rewards.types","modules/src.environment.rewards.types.manual_zone","modules/src.environment.rewards.wrappers","modules/src.environment.scenarios","modules/src.environment.scenarios.types","modules/src.environment.services","modules/src.environment.utils","modules/src.environment.wrappers"],envversion:{"sphinx.domains.c":2,"sphinx.domains.changeset":1,"sphinx.domains.citation":1,"sphinx.domains.cpp":4,"sphinx.domains.index":1,"sphinx.domains.javascript":2,"sphinx.domains.math":2,"sphinx.domains.python":3,"sphinx.domains.rst":2,"sphinx.domains.std":2,sphinx:56},filenames:["index.rst","installation/basic_install.md","installation/full_install.md","intro/features.md","intro/making_a_new_scene.md","intro/what_is_social_gym.md","modules/modules.rst","modules/src.rst","modules/src.environment.rst","modules/src.environment.callbacks.rst","modules/src.environment.environment_types.rst","modules/src.environment.extractors.rst","modules/src.environment.observations.rst","modules/src.environment.observations.types.rst","modules/src.environment.observations.types.manual_zone.rst","modules/src.environment.rewards.rst","modules/src.environment.rewards.types.rst","modules/src.environment.rewards.types.manual_zone.rst","modules/src.environment.rewards.wrappers.rst","modules/src.environment.scenarios.rst","modules/src.environment.scenarios.types.rst","modules/src.environment.services.rst","modules/src.environment.utils.rst","modules/src.environment.wrappers.rst"],objects:{"":[[7,0,0,"-","src"]],"src.config_run":[[7,1,1,"","kinds"],[7,3,1,"","run"]],"src.config_run.kinds":[[7,2,1,"","ao"],[7,2,1,"","eo"],[7,2,1,"","sacadrl"]],"src.environment":[[9,0,0,"-","callbacks"],[10,0,0,"-","environment_types"],[11,0,0,"-","extractors"],[12,0,0,"-","observations"],[15,0,0,"-","rewards"],[8,0,0,"-","ros_social_gym"],[19,0,0,"-","scenarios"],[21,0,0,"-","services"],[22,0,0,"-","utils"],[23,0,0,"-","wrappers"]],"src.environment.callbacks":[[9,0,0,"-","callbacks"]],"src.environment.callbacks.callbacks":[[9,1,1,"","EvalCallback"],[9,1,1,"","OptunaCallback"]],"src.environment.callbacks.callbacks.EvalCallback":[[9,4,1,"","update_child_locals"]],"src.environment.environment_types":[[10,0,0,"-","manual_zone"]],"src.environment.environment_types.manual_zone":[[10,1,1,"","ManualZoneEnv"],[10,1,1,"","ManualZoneUTMRSResponse"],[10,1,1,"","RvisZoneVisualization"]],"src.environment.environment_types.manual_zone.ManualZoneEnv":[[10,2,1,"","agents_priority_orders"],[10,4,1,"","reset"],[10,4,1,"","sim_step"]],"src.environment.environment_types.manual_zone.ManualZoneUTMRSResponse":[[10,2,1,"","agents_current_order"],[10,2,1,"","agents_priority_order"],[10,2,1,"","in_zone"],[10,4,1,"","process"],[10,4,1,"","set_zone_vars"]],"src.environment.environment_types.manual_zone.RvisZoneVisualization":[[10,4,1,"","point"],[10,4,1,"","publish"],[10,4,1,"","rect"]],"src.environment.extractors":[[11,0,0,"-","lstm_agent_obs"]],"src.environment.extractors.lstm_agent_obs":[[11,1,1,"","LSTMAgentObs"]],"src.environment.extractors.lstm_agent_obs.LSTMAgentObs":[[11,4,1,"","forward"],[11,4,1,"","make_tensors"]],"src.environment.observations":[[12,0,0,"-","common_observations"],[12,0,0,"-","observation"],[12,0,0,"-","observer"],[13,0,0,"-","types"]],"src.environment.observations.common_observations":[[12,3,1,"","dsacadrl_observations"]],"src.environment.observations.observation":[[12,1,1,"","Observation"]],"src.environment.observations.observation.Observation":[[12,4,1,"","name"],[12,4,1,"","observations"]],"src.environment.observations.observer":[[12,1,1,"","Observer"]],"src.environment.observations.observer.Observer":[[12,4,1,"","arr_to_dict"],[12,4,1,"","make_observation"],[12,2,1,"","registered_observations"],[12,4,1,"","reset"],[12,4,1,"","setup"]],"src.environment.observations.types":[[13,0,0,"-","agents_goal_distance"],[13,0,0,"-","agents_heading_direction"],[13,0,0,"-","agents_others_distance"],[13,0,0,"-","agents_pose"],[13,0,0,"-","agents_preferred_velocity"],[13,0,0,"-","agents_velocity"],[13,0,0,"-","collision_observation"],[14,0,0,"-","manual_zone"],[13,0,0,"-","other_agent_observables"],[13,0,0,"-","others_heading_direction"],[13,0,0,"-","others_poses"],[13,0,0,"-","others_velocities"],[13,0,0,"-","success_observation"]],"src.environment.observations.types.agents_goal_distance":[[13,1,1,"","AgentsGoalDistance"]],"src.environment.observations.types.agents_goal_distance.AgentsGoalDistance":[[13,2,1,"","history"],[13,2,1,"","history_length"],[13,4,1,"","name"]],"src.environment.observations.types.agents_heading_direction":[[13,1,1,"","AgentsHeadingDirection"]],"src.environment.observations.types.agents_heading_direction.AgentsHeadingDirection":[[13,4,1,"","name"]],"src.environment.observations.types.agents_others_distance":[[13,1,1,"","AgentsOthersDistance"]],"src.environment.observations.types.agents_others_distance.AgentsOthersDistance":[[13,4,1,"","name"],[13,2,1,"","num_others"]],"src.environment.observations.types.agents_pose":[[13,1,1,"","AgentsPose"]],"src.environment.observations.types.agents_pose.AgentsPose":[[13,4,1,"","name"]],"src.environment.observations.types.agents_preferred_velocity":[[13,1,1,"","AgentsPreferredVelocity"]],"src.environment.observations.types.agents_preferred_velocity.AgentsPreferredVelocity":[[13,4,1,"","name"]],"src.environment.observations.types.agents_velocity":[[13,1,1,"","AgentsVelocity"]],"src.environment.observations.types.agents_velocity.AgentsVelocity":[[13,2,1,"","history"],[13,2,1,"","history_length"],[13,2,1,"","ignore_theta"],[13,4,1,"","name"]],"src.environment.observations.types.collision_observation":[[13,1,1,"","CollisionObservation"]],"src.environment.observations.types.collision_observation.CollisionObservation":[[13,4,1,"","name"]],"src.environment.observations.types.manual_zone":[[14,0,0,"-","agent_in_zone"],[14,0,0,"-","agent_zone_current_order"],[14,0,0,"-","agent_zone_priority_order"]],"src.environment.observations.types.manual_zone.agent_in_zone":[[14,1,1,"","AgentInZone"]],"src.environment.observations.types.manual_zone.agent_in_zone.AgentInZone":[[14,4,1,"","name"]],"src.environment.observations.types.manual_zone.agent_zone_current_order":[[14,1,1,"","AgentZoneCurrentOrder"]],"src.environment.observations.types.manual_zone.agent_zone_current_order.AgentZoneCurrentOrder":[[14,4,1,"","name"]],"src.environment.observations.types.manual_zone.agent_zone_priority_order":[[14,1,1,"","AgentZonePriorityOrder"]],"src.environment.observations.types.manual_zone.agent_zone_priority_order.AgentZonePriorityOrder":[[14,4,1,"","name"]],"src.environment.observations.types.other_agent_observables":[[13,1,1,"","OtherAgentObservables"]],"src.environment.observations.types.other_agent_observables.OtherAgentObservables":[[13,4,1,"","name"],[13,2,1,"","num_others"]],"src.environment.observations.types.others_heading_direction":[[13,1,1,"","OthersHeadingDirection"]],"src.environment.observations.types.others_heading_direction.OthersHeadingDirection":[[13,4,1,"","name"],[13,2,1,"","num_others"]],"src.environment.observations.types.others_poses":[[13,1,1,"","OthersPoses"]],"src.environment.observations.types.others_poses.OthersPoses":[[13,4,1,"","name"],[13,2,1,"","num_others"]],"src.environment.observations.types.others_velocities":[[13,1,1,"","OthersVelocities"]],"src.environment.observations.types.others_velocities.OthersVelocities":[[13,4,1,"","name"],[13,2,1,"","num_others"]],"src.environment.observations.types.success_observation":[[13,1,1,"","SuccessObservation"]],"src.environment.observations.types.success_observation.SuccessObservation":[[13,4,1,"","name"]],"src.environment.rewards":[[15,0,0,"-","common_rewards"],[15,0,0,"-","reward"],[15,0,0,"-","rewarder"],[16,0,0,"-","types"],[18,0,0,"-","wrappers"]],"src.environment.rewards.common_rewards":[[15,3,1,"","dsacadrl_rewards"]],"src.environment.rewards.reward":[[15,1,1,"","Reward"]],"src.environment.rewards.reward.Reward":[[15,4,1,"","name"],[15,4,1,"","score"],[15,2,1,"","weight"]],"src.environment.rewards.rewarder":[[15,1,1,"","Rewarder"]],"src.environment.rewards.rewarder.Rewarder":[[15,2,1,"","registered_rewards"],[15,4,1,"","reset"],[15,4,1,"","reward"],[15,4,1,"","setup"]],"src.environment.rewards.types":[[16,0,0,"-","collisions"],[16,0,0,"-","existence_penalty"],[16,0,0,"-","goal_distance"],[16,0,0,"-","goal_distance_change"],[17,0,0,"-","manual_zone"],[16,0,0,"-","preferred_velocity"],[16,0,0,"-","social_norm_cross"],[16,0,0,"-","social_norm_overtake"],[16,0,0,"-","social_norm_pass"],[16,0,0,"-","success"],[16,0,0,"-","velocity_control"]],"src.environment.rewards.types.collisions":[[16,1,1,"","Collisions"]],"src.environment.rewards.types.collisions.Collisions":[[16,4,1,"","name"]],"src.environment.rewards.types.existence_penalty":[[16,1,1,"","ExistencePenalty"]],"src.environment.rewards.types.existence_penalty.ExistencePenalty":[[16,4,1,"","name"]],"src.environment.rewards.types.goal_distance":[[16,1,1,"","GoalDistance"]],"src.environment.rewards.types.goal_distance.GoalDistance":[[16,4,1,"","name"]],"src.environment.rewards.types.goal_distance_change":[[16,1,1,"","GoalDistanceChange"]],"src.environment.rewards.types.goal_distance_change.GoalDistanceChange":[[16,4,1,"","name"]],"src.environment.rewards.types.manual_zone":[[17,0,0,"-","enforced_order"]],"src.environment.rewards.types.manual_zone.enforced_order":[[17,1,1,"","EnforcedOrder"]],"src.environment.rewards.types.manual_zone.enforced_order.EnforcedOrder":[[17,4,1,"","name"],[17,2,1,"","on_enter"],[17,2,1,"","on_exit"],[17,2,1,"","previous_zone_state"]],"src.environment.rewards.types.preferred_velocity":[[16,1,1,"","PreferredVelocity"]],"src.environment.rewards.types.preferred_velocity.PreferredVelocity":[[16,4,1,"","name"]],"src.environment.rewards.types.social_norm_cross":[[16,1,1,"","SocialNormCross"]],"src.environment.rewards.types.social_norm_cross.SocialNormCross":[[16,2,1,"","dist_to_other_threshold"],[16,2,1,"","goal_dist_threshold"],[16,2,1,"","heading_angle_thresholds"],[16,4,1,"","name"],[16,2,1,"","relative_agent_to_other_angle_threshold"]],"src.environment.rewards.types.social_norm_overtake":[[16,1,1,"","SocialNormOvertake"]],"src.environment.rewards.types.social_norm_overtake.SocialNormOvertake":[[16,2,1,"","goal_dist_threshold"],[16,2,1,"","heading_angle_threshold"],[16,4,1,"","name"],[16,2,1,"","px_threshold"],[16,2,1,"","py_threshold"]],"src.environment.rewards.types.social_norm_pass":[[16,1,1,"","SocialNormPass"]],"src.environment.rewards.types.social_norm_pass.SocialNormPass":[[16,2,1,"","goal_dist_threshold"],[16,2,1,"","heading_angle_threshold"],[16,4,1,"","name"],[16,2,1,"","px_threshold"],[16,2,1,"","py_threshold"]],"src.environment.rewards.types.success":[[16,1,1,"","Success"]],"src.environment.rewards.types.success.Success":[[16,4,1,"","name"]],"src.environment.rewards.types.velocity_control":[[16,1,1,"","VelocityControl"]],"src.environment.rewards.types.velocity_control.VelocityControl":[[16,4,1,"","name"]],"src.environment.rewards.wrappers":[[18,0,0,"-","linear_weight_scheduler"]],"src.environment.rewards.wrappers.linear_weight_scheduler":[[18,1,1,"","LinearWeightScheduler"]],"src.environment.rewards.wrappers.linear_weight_scheduler.LinearWeightScheduler":[[18,4,1,"","name"]],"src.environment.ros_social_gym":[[8,1,1,"","AgentColor"],[8,1,1,"","RosSocialEnv"]],"src.environment.ros_social_gym.RosSocialEnv":[[8,4,1,"","action_space"],[8,4,1,"","calculate_reward"],[8,4,1,"","close"],[8,4,1,"","default_action"],[8,2,1,"","env_response_type"],[8,2,1,"","launch_config"],[8,4,1,"","make_observation"],[8,2,1,"","metadata"],[8,4,1,"","new_scenario"],[8,2,1,"","num_agents"],[8,2,1,"","num_humans"],[8,4,1,"","observation_space"],[8,2,1,"","observer"],[8,4,1,"","render"],[8,4,1,"","reset"],[8,2,1,"","rewarder"],[8,4,1,"","seed"],[8,4,1,"","sim_step"],[8,4,1,"","state"],[8,4,1,"","step"]],"src.environment.scenarios":[[19,0,0,"-","common_scenarios"],[19,0,0,"-","scenario"],[20,0,0,"-","types"]],"src.environment.scenarios.common_scenarios":[[19,3,1,"","closed_door_1__opposing_sides"],[19,3,1,"","closed_door_1__same_goals"],[19,3,1,"","elevator_loading"],[19,3,1,"","exp1_train_scenario"],[19,3,1,"","exp2_train_scenario"]],"src.environment.scenarios.scenario":[[19,1,1,"","Scenario"]],"src.environment.scenarios.scenario.Scenario":[[19,4,1,"","generate_scenario"],[19,4,1,"","load_nav_nodes"],[19,4,1,"","make_scenario"]],"src.environment.scenarios.types":[[20,0,0,"-","cycle"],[20,0,0,"-","graph_nav"],[20,0,0,"-","manual"]],"src.environment.scenarios.types.cycle":[[20,1,1,"","CycleScenario"]],"src.environment.scenarios.types.cycle.CycleScenario":[[20,4,1,"","generate_scenario"]],"src.environment.scenarios.types.graph_nav":[[20,1,1,"","GraphNavScenario"]],"src.environment.scenarios.types.graph_nav.GraphNavScenario":[[20,4,1,"","generate_scenario"]],"src.environment.scenarios.types.manual":[[20,1,1,"","ManualScenario"]],"src.environment.scenarios.types.manual.ManualScenario":[[20,4,1,"","generate_scenario"]],"src.environment.services":[[21,0,0,"-","utmrs"]],"src.environment.services.utmrs":[[21,1,1,"","UTMRS"],[21,1,1,"","UTMRSResponse"]],"src.environment.services.utmrs.UTMRS":[[21,4,1,"","reset"],[21,4,1,"","step"]],"src.environment.services.utmrs.UTMRSResponse":[[21,2,1,"","collision"],[21,2,1,"","done"],[21,2,1,"","door_pose"],[21,2,1,"","door_state"],[21,2,1,"","follow_target"],[21,2,1,"","goal_pose"],[21,2,1,"","human_poses"],[21,2,1,"","human_vels"],[21,2,1,"","local_target"],[21,2,1,"","other_robot_poses"],[21,2,1,"","other_robot_vels"],[21,4,1,"","process"],[21,2,1,"","robot_poses"],[21,2,1,"","robot_state"],[21,2,1,"","robot_vels"],[21,4,1,"","set_vars"],[21,2,1,"","success"]],"src.environment.utils":[[22,0,0,"-","evaluate_policy"],[22,0,0,"-","utils"]],"src.environment.utils.evaluate_policy":[[22,3,1,"","evaluate_policy"]],"src.environment.utils.utils":[[22,1,1,"","LogFilter"],[22,3,1,"","filter_stdout"],[22,3,1,"","get_tboard_writer"],[22,3,1,"","poses_to_np_array"]],"src.environment.utils.utils.LogFilter":[[22,4,1,"","flush"],[22,4,1,"","write"]],"src.environment.wrappers":[[23,0,0,"-","collision_episode_ender"],[23,0,0,"-","entropy_episode_ender"],[23,0,0,"-","new_scenario_wrapper"],[23,0,0,"-","reward_stripper"],[23,0,0,"-","tensorboard_writer"],[23,0,0,"-","time_limit"]],"src.environment.wrappers.collision_episode_ender":[[23,1,1,"","CollisionEpisodeEnder"]],"src.environment.wrappers.collision_episode_ender.CollisionEpisodeEnder":[[23,4,1,"","seed"],[23,4,1,"","step"]],"src.environment.wrappers.entropy_episode_ender":[[23,1,1,"","EntropyEpisodeEnder"]],"src.environment.wrappers.entropy_episode_ender.EntropyEpisodeEnder":[[23,2,1,"","agent_positions"],[23,4,1,"","calculate_deltas"],[23,2,1,"","distance_delta"],[23,2,1,"","negative_multiplier_only"],[23,4,1,"","reset"],[23,4,1,"","reset_positions"],[23,2,1,"","reward_multiplier"],[23,4,1,"","seed"],[23,4,1,"","step"],[23,2,1,"","timestep_threshold"]],"src.environment.wrappers.new_scenario_wrapper":[[23,1,1,"","NewScenarioWrapper"]],"src.environment.wrappers.new_scenario_wrapper.NewScenarioWrapper":[[23,2,1,"","env"],[23,2,1,"","episode_count"],[23,2,1,"","new_scenario_episode_frequency"],[23,2,1,"","plans"],[23,4,1,"","reset"],[23,4,1,"","seed"]],"src.environment.wrappers.reward_stripper":[[23,1,1,"","RewardStripper"]],"src.environment.wrappers.reward_stripper.RewardStripper":[[23,4,1,"","seed"],[23,4,1,"","step"]],"src.environment.wrappers.tensorboard_writer":[[23,1,1,"","TensorboardWriter"]],"src.environment.wrappers.tensorboard_writer.TensorboardWriter":[[23,4,1,"","image_callback"],[23,4,1,"","reset"],[23,4,1,"","seed"],[23,4,1,"","step"]],"src.environment.wrappers.time_limit":[[23,1,1,"","TimeLimitWrapper"]],"src.environment.wrappers.time_limit.TimeLimitWrapper":[[23,4,1,"","reset"],[23,4,1,"","seed"],[23,4,1,"","step"]],"src.multi_agent_optuna":[[7,3,1,"","objective"]],src:[[7,0,0,"-","config_run"],[8,0,0,"-","environment"],[7,0,0,"-","multi_agent_optuna"]]},objnames:{"0":["py","module","Python module"],"1":["py","class","Python class"],"2":["py","attribute","Python attribute"],"3":["py","function","Python function"],"4":["py","method","Python method"]},objtypes:{"0":"py:module","1":"py:class","2":"py:attribute","3":"py:function","4":"py:method"},terms:{"0":[0,2,7,8,15,16,17,18,20,22],"017613":2,"08862":16,"1":[3,7,8,9,13,15,16,17,18,20],"10":[2,7,16,22],"100":7,"1000":18,"100000":7,"10m":2,"11":[2,16],"11011":[],"11_20_22":1,"12":[2,16],"12345":2,"13":2,"14":2,"15":[1,2],"15m":[],"16":2,"1671386774":2,"168961":2,"17":2,"1703":16,"18":2,"188148416":2,"19":2,"192522299":2,"199740381":2,"2":[0,7,16],"20":2,"2000":7,"20221218_multi_agent_finishing_and_cleanup":1,"204534827":2,"21":2,"2109":[],"22":2,"2222":2,"2223":2,"25":[7,20],"25000":7,"26":2,"2d":[1,5],"2nd":2,"3":16,"30m":1,"34904946":22,"356194490192345":16,"4":16,"40":2,"402":22,"5":[16,20],"6":2,"6hour":2,"7":2,"7853981633974483":16,"8":[1,2],"9":2,"abstract":[5,12,15],"case":2,"class":[3,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],"default":[2,22],"do":[1,2,5,22],"final":2,"float":[7,13,15,16,17,18,22,23],"function":[3,22],"import":[2,3,4,5],"int":[7,8,10,11,13,18,20,21,22,23],"long":2,"new":[0,2,9,20,22],"return":[12,13,15,16,22],"static":[4,13,22],"true":[2,7,13,17,19,20,22],"try":[1,2],"var":2,"while":1,A:[3,8,12,15,20],For:2,If:[1,2,22],In:[1,2],It:[1,2,5,9],NOT:1,Of:2,The:[0,1,3,4,9,12,15,16,17,18,22],Then:2,There:2,These:4,To:[1,9],Will:12,__len__:12,ab:[],abc:[12,15,18],abil:2,abl:2,about:[2,8,22],abov:[2,22],abrupt:16,absolut:23,accord:[9,15],account:9,achiev:18,action:[9,22,23],action_dict:8,action_spac:8,activ:1,actual:[2,13],ad:1,add:2,addit:22,affect:22,after:[1,2,9,22],again:2,agent:[2,3,4,5,8,9,12,13,14,15,16,22,23],agent_goal_distance_ob:7,agent_in_zon:[12,13],agent_path:20,agent_pose_ignore_theta:7,agent_pose_ob:7,agent_posit:23,agent_velocity_ignore_theta:7,agent_velocity_ob:7,agent_zone_current_ord:[12,13],agent_zone_priority_ord:[12,13],agentcolor:8,agentinzon:14,agents_current_ord:10,agents_goal_dist:[8,12],agents_heading_direct:[8,12],agents_others_dist:[8,12],agents_pos:[8,12],agents_preferred_veloc:[8,12],agents_priority_ord:10,agents_veloc:[8,12],agentsgoaldist:13,agentsheadingdirect:13,agentsothersdist:13,agentspos:13,agentspreferredveloc:13,agentsveloc:13,agentzonecurrentord:14,agentzonepriorityord:14,all:[1,2,5,12,15,20,22,23],all_config:[19,20],allow:[1,2,5],allow_any_ord:17,allow_human:13,allow_other_robot:13,allowed_agent_goal_posit:20,allowed_agent_start_posit:20,allowed_human_goal_posit:20,allowed_human_start_posit:20,almost:[2,8],alreadi:2,also:[2,3,5],alter:22,although:[1,3,22],am:2,amount:15,amrl:[],amrl_map:2,amrl_msg:2,an:[1,2,3,4,9,12,22,23],ani:[2,8,9,10,11,12,22,23],anyth:[2,22],anywher:2,ao:7,api:5,appear:[1,22],appli:2,ar:[0,2,3,4,22,23],arg:[8,9,10,11,12,13,15,16,17,18,21,23],argument:2,around:[1,2],arr_to_dict:12,arrai:[12,15,21,22],art:2,arxiv:16,assum:[1,2],autom:2,avail:2,averag:22,avoid:[12,22],awhil:2,b:8,back:2,base:[7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],base_class:22,basealgorithm:22,basecallback:9,basefeaturesextractor:11,baselin:[3,5,22],baselines3:22,baseparallelwrap:23,basic:0,becaus:[1,5],been:[9,22],befor:[2,15,22],being:[2,12,15,22],below:[2,3,4],best:9,best_model_save_path:9,between:[13,18],bia:22,big:2,bin:[1,2],bit:[2,4,22],blue:[1,4],bool:[7,10,13,16,17,19,20,21,22,23],both:1,bottom:2,box:2,branch:[],breakpoint:2,broken:2,buckl:0,bug:1,build:12,built:[2,5],bunch:[],button:2,c:[1,2,5],calcul:15,calculate_delta:23,calculate_reward:8,call:[9,15,22],callabl:22,callback:[7,8,22],callback_after_ev:9,callback_on_new_best:9,can:[1,2,3,9,12,22,23],cannot:[1,4],catkin_mak:2,cd:[1,2],challeng:1,chang:[2,3,16,22],check:[1,2,22],checkout:[],choos:[2,20],classmethod:[10,12,13,14,15,16,17,18,21],classvar:8,clean:2,click:2,clion:[],clip:16,clipboard:2,clock:[],close:[1,2,8],closed_door_1__opposing_sid:19,closed_door_1__same_go:19,cmake:2,code:[1,2],cog:2,collect:[9,13],collid:[16,23],collis:[8,13,15,21],collision_end:7,collision_episode_end:[7,8],collision_ob:7,collision_observ:[8,12],collision_penalti:7,collision_penalty_scale_dur:7,collisionepisodeend:23,collisionobserv:13,color:3,com:[2,22],come:22,command:[1,2],comment:2,commerci:2,commit:1,common:[1,9,11,22],common_observ:[7,8],common_reward:[7,8],common_scenario:[7,8],commun:1,compil:2,complet:16,complex:5,compos:2,config:[0,2,15,19],config_read:2,config_run:[2,6],config_runn:[1,19,20],configur:[1,2],congratul:2,connect:2,consist:4,contact:2,contain:[1,3,12,22],content:6,continu:17,continue_from:7,contribut:1,control:3,convert:22,copi:2,correct:12,correctli:[1,2],correspond:[9,11],could:2,count:[2,22],cours:2,crash:2,creat:[2,3],create_env_templ:1,credenti:2,critic:1,cross:16,cube:2,cuda:7,current:[2,12,13,14,15],custom:[1,2],cycl:[8,19],cyclescenario:20,d:12,data:[12,22],data_map:15,debug:[1,2,7,20],debugg:[1,2],decreas:18,default_act:8,defin:16,delai:2,delet:2,delta:23,denot:4,depend:8,deploi:2,deploy:2,dequ:13,detail:22,determinist:[9,22],develop:[1,2,3,5],devic:7,dict:[9,12,15,22,23],dictionari:15,did:2,die:23,differ:[2,22],difficult:2,direct:13,directori:2,discuss:22,displai:[],dist_to_other_threshold:16,distanc:[13,16],distance_delta:23,dive:2,divid:22,divis:22,dlr:22,doc:[1,2],docker:[1,3],dockerfil:2,docremotetest:2,document:[1,8],doe:2,don:2,done:[2,5,21,22],door:2,door_pos:21,door_stat:21,doubl:2,down:2,drop:2,dropdown:2,dsacadrl_observ:12,dsacadrl_reward:15,durat:18,dure:[2,3,9],dynam:5,e:[2,9,16,22],each:[1,2,3,5,9,13,15,22,23],earli:22,eas:1,easi:[1,2,3,5,18,19],easier:[2,3],edg:2,edit:2,editor:2,effect:9,element:22,elevator_load:19,els:22,empti:2,end:[2,23],ending_eval_tri:7,ending_eval_with_best:7,enforced_ord:[15,16],enforced_order_penalty_for_incorrect_ord:7,enforced_order_reward:7,enforced_order_track_exit:7,enforced_order_weak_signal_out_of_zon:7,enforcedord:17,enhanc:3,enough:9,ensur:[1,2],entir:[2,8],entropy_constant_penalti:7,entropy_constant_penalty_only_those_that_did_not_finish:7,entropy_end:7,entropy_episode_end:[7,8],entropy_max_dist:7,entropy_max_timestep:7,entropy_multiply_negative_rewards_onli:7,entropy_reward:7,entropy_reward_multipli:7,entropyepisodeend:23,env:[2,9,10,12,15,16,22,23],env_nam:[19,20],env_respons:[10,12,15,21],env_response_typ:8,environ:[2,3,4,5,6,7],environment_cr:8,environment_typ:[7,8],eo:7,episod:[9,13,15,20,22,23],episode_count:23,eq:16,error:[2,22],esc:1,especi:2,etc:[2,5],eval:[7,9],eval_env:9,eval_freq:9,eval_frequ:7,evalcallback:9,evalu:[1,3,5,9,22],evaluate_polici:[7,8,9],eventcallback:9,eventu:2,everi:[2,9,13,16],everyth:1,exactli:2,except:2,execut:2,exist:2,existence_penalti:[7,8,15],existencepenalti:16,exit:2,exp1:1,exp1_train_scenario:19,exp2:[2,7],exp2_train_scenario:19,expect:[1,2,9,22],experi:[1,3,5],experiment:5,experiment_nam:7,explain:4,explor:2,extend:5,extract:[3,11],extractor:[7,8],ezpickl:8,fail:[1,2],failur:1,fals:[7,8,10,13,16,17,19,20,22,23],faq:0,farama:8,faster:2,featur:[0,1,11],features_dim:11,few:2,file:[2,20],fill:[1,2],filter:22,filter_stdout:22,find:2,fine:2,finish:2,fire:2,first:[1,2,4,22],fix:1,flavor:1,flush:22,fn:22,focu:2,focus:3,folder:9,follow:1,follow_target:21,form:2,forward:[4,11],found:[2,3],foundat:5,from:[0,1,5,8,12,15,22],full:[0,1,3],fun:2,g:[8,22],gener:12,generate_scenario:[19,20],geometri:2,get:[1,2,22],get_tboard_writ:22,get_var:2,git:[1,2],github:[2,22],given:[12,15,22],global:[2,22],go:[1,2],goal:[13,16],goal_dist:[8,15],goal_dist_threshold:16,goal_distance_chang:[8,15],goal_distance_reward:7,goal_distance_reward_clip:7,goal_pos:21,goaldist:16,goaldistancechang:16,goalon:2,goe:1,good:[2,9],graph:[3,20],graph_nav:[8,19],graphnavscenario:20,grid:1,guid:0,gym:[8,11,22,23],gym_gen:2,ha:[1,2,3,9,16,22],hallwai:1,halt:2,handl:[2,15,23],hang:[1,2],hard:[2,5],have:[2,22],haven:2,head:13,heading_angle_threshold:16,heavili:2,hei:[],help:3,helper:[3,12,15],here:[1,2,3],histori:13,history_length:13,hit:[1,2],home:2,host:[1,2],hour:2,how:[2,22],howev:[1,2,5],http:[2,8,16,22],human:[8,13],human_path:20,human_pos:21,human_vel:21,i:[9,16],icon:2,id:20,idea:2,ignore_theta:13,image_callback:23,implement:[3,5],improv:3,in_zon:10,includ:[2,3],incom:2,incorrect:2,incorrect_penalti:17,increas:18,increment:18,indepth:1,index:0,indic:3,individu:15,info:2,inform:23,init:1,initi:[2,9],input:2,insid:1,inspir:2,instal:0,instanti:15,instead:22,intention:2,intermediate_eval_tri:7,intern:1,interpert:[],interpol:18,interpret:2,invoc:2,issu:[2,22],item:3,job:1,json:[1,2],just:[1,2,13],keep:[2,15,16,17,18],kei:15,kick:1,kind:7,know:[2,5,20],knowledg:15,known:1,kwarg:[8,9,10,11,12,13,15,16,17,18,21,23],label:3,lack:22,last:11,later:1,launch_config:8,layer:11,least:2,led:16,left:2,length:[12,22],let:0,level:[2,19],librari:2,lidar:5,life:3,lightweight:5,like:[1,2,5],limit:23,line:[1,2,4],linear_weight_schedul:[8,15],linearli:18,linearweightschedul:18,link:3,linux:1,list:[2,3,10,12,15,20,21,22,23],littl:2,live:2,ll:[1,2],load:2,load_nav_nod:19,local:[2,7,9,22],local_target:21,localhost:2,locals_:9,locat:2,log:[2,15,22],log_nam:22,log_path:9,logfilt:22,logger:15,lol:2,longer:23,look:[1,2],loop:[],lot:[3,8],low:2,lstm:[],lstm_agent_ob:[7,8],lstmagentob:11,lua:2,mac:1,machin:[1,2],made:[2,12],mai:[1,2],main:[],major:22,make:[2,3,5,22],make_observ:[8,12],make_scenario:19,make_tensor:11,mani:1,manual:[2,8,19],manual_zon:[7,8,12,13,15,16],manualscenario:20,manualzoneenv:10,manualzoneutmrsrespons:10,map:[2,5],max:[3,9,18,23],max_step:23,max_weight:18,maximum:23,mayb:2,mean:[2,22],mean_reward:9,memor:2,messag:2,met:22,metadata:8,method:8,might:2,mimick:[],min_weight:18,minimum:[9,22],mismatch:12,miss:2,mlppolici:7,mode:[2,8],model:[3,9,22],modifi:22,modul:[0,6],modular:3,monitor:[7,9,22],more:[1,2,5,22],most:[2,3],move:[2,4,23],ms:2,msg:23,much:2,multi:[2,3,5],multi_agent_optuna:6,multiag:[3,8],multipl:[8,9],must:9,my:2,n:[1,22],n_env:9,n_eval_episod:[9,22],name:[1,2,8,12,13,14,15,16,17,18],nan:2,narrow:1,nav:[2,20],nav_path:19,navig:[1,3,4,5,8],ndarrai:[8,23],necessari:1,need:[1,2,3,5,12],negative_multiplier_onli:23,network:[1,12],new_scenario:8,new_scenario_episode_frequ:23,new_scenario_wrapp:[7,8],newscenariowrapp:23,next:[1,2,4],nocoinman:2,node:[2,20],noetic:2,none:[7,8,9,10,19,20,21,22,23],norm:[13,16],note:[1,2,5,22],now:[1,2],np:[12,22],npz:9,num_ag:[7,8,20],num_human:[8,20],num_oth:13,number:[3,9,11,13,22,23],numpi:[8,12,15,21,22,23],object:[4,7,8,10,12,15,19,21,22],obs_dict:11,obs_map:8,observ:[3,7,8,11,15],observation_map:15,observation_spac:[8,11],obstacl:5,off:1,often:[2,15,16,17,18],ok:2,okai:2,on_ent:17,on_exit:17,onc:[2,9],one:2,onli:[1,2,18],onto:22,ontop:5,open:1,openai:8,oper:[1,5],opt:2,option:[1,2,7,8,10,17,19,20,22,23],optunacallback:9,order:14,org:[8,16],origin:0,other:[1,2,5,10,13,15,16,17,18,22],other_agent_observ:[8,12],other_poses_actual_posit:7,other_poses_ignore_theta:7,other_poses_ob:7,other_robot_pos:21,other_robot_vel:21,other_velocities_ignore_theta:7,other_velocities_ob:7,otheragentobserv:13,others_heading_direct:[8,12],others_pos:[8,12],others_veloc:[8,12],othersheadingdirect:13,otherspos:13,othersveloc:13,otherwis:1,our:[1,2],out:[1,2],output:2,over:[1,3,23],overal:23,overtak:16,own:[1,2,3],p:2,packag:[2,6],pad:12,page:[0,1,16],pair:15,panel:[1,2],paper:3,parallelenv:8,paramet:[1,2,9,11,12,15,22,23],part:[],partial:3,partially_observ:[7,19,20],pass:[2,9,12,16,22],password:2,past:2,path:[1,2,4,9],pdf:16,pedsim:2,pedsim_ro:2,penal:15,penalti:[15,16,17,18],per:[9,16,20,22,23],perform:[9,22],perman:3,pettingzoo:[3,5,8,22,23],pink:[1,4],pip:[1,2],place:[2,15,16,17,18],plai:2,plan:[20,23],plug:2,point:[2,10],polici:[3,12,22],policy_algo_kwarg:7,policy_algo_nam:7,policy_algo_sb3_contrib:7,policy_nam:7,port:2,pos_i:13,pos_theta:13,pos_x:13,pose:[1,13,22],poses_to_np_arrai:22,posit:13,ppo:7,prefer:13,preferred_veloc:[8,13,15],preferredveloc:16,present:15,pretti:[2,4],previou:2,previous_zone_st:17,probabl:18,problem:1,process:[1,2,3,10,21],produc:2,profession:2,profil:2,program:2,project:1,project_root:[1,2],promis:2,publish:[2,10],punish:16,purpl:4,px_threshold:16,py:[1,2],py_threshold:16,pycharm:[],python:[5,22],pythonript:[],qstandardpath:2,qualiti:3,question:22,queue:2,quit:2,r:[1,2,8],rais:22,ran:2,rate:2,re:[1,2,8],re_pattern:22,reach:9,read:2,readi:2,realli:[2,13],rebuild:2,recommend:1,record:[3,23],rect:10,recurs:1,refer:[2,9,22],referenc:1,refresh:2,regist:15,registered_observ:12,registered_reward:15,relative_agent_to_other_angle_threshold:16,releas:0,relev:[2,12],remain:3,rememb:2,remot:2,remotedebugg:2,remov:[2,22,23],renam:2,render:[8,9,22],render_mod:8,repeat:[1,2],repres:15,requir:[0,3,15],rerun:2,reset:[8,10,12,15,21,22,23],reset_posit:23,respons:[12,15],restart:1,result:22,return_episode_reward:22,return_info:[8,10,23],reward:[3,7,8,9,22,23],reward_multipli:23,reward_stripp:[7,8],reward_threshold:[9,22],rewardstripp:23,right:2,rl:22,rm:[2,22],ro:[1,5,8],robot:[1,2,5],robot_idx:[10,21],robot_ob:21,robot_pos:21,robot_st:21,robot_vel:21,rollout:9,room:1,root:[],root_directori:1,ros_master_uri:2,ros_root:2,ros_social_env:8,ros_social_gym:[6,7,10,12,15,23],rosdep:2,rosdev:2,rosedep:2,rossocialenv:[8,10,12,15,23],run:[7,15,22],run_config:1,run_nam:7,run_typ:7,runner:0,runtim:2,rvi:[1,2],rviszonevisu:10,rviz:8,s:2,sacadrl:[1,7,12],sai:2,same:[1,2],save:[1,2,9],sb3_contrib:2,scalar:15,scale:22,scenario:[5,7,8],scene:[2,3],scene_config:2,score:15,script:1,search:0,second:[1,2,22],see:[1,2,22],seed:[8,10,23],select:2,separ:1,server:2,servic:[2,7,8,10,12],service_mod:2,set:[1,3,23],set_path:2,set_var:21,set_zone_var:10,setup:[1,12,15],sh:[1,2],shape:12,share:2,shell:2,should:[1,2,9],shoutout:2,show:1,shown:2,side:1,sim:[12,15],sim_config:2,sim_step:[8,10],similar:[2,5,22],simpl:[16,20],simul:[2,3,5,15],sinc:2,singl:[3,5],size:2,slightli:22,small:[2,3],so:[1,2],sobumodul:2,social:[8,16],social_gym:2,social_gym_1:[],social_nav:2,social_norm_cross:[8,15],social_norm_overtak:[8,15],social_norm_pass:[8,15],socialgym:22,socialnormcross:16,socialnormovertak:16,socialnormpass:16,some:[1,2,23],someth:[2,16],somewhat:2,sourc:[1,2],space:[11,12],special:1,specif:[],specifi:[1,12],speedup_factor:2,src:[],sript:[],ssh:[],stabl:[3,5,22],stable_baselines3:[9,11,22],stablebaselin:22,stack:0,stackoverflow:22,start:[1,2,18],starting_collision_penalti:7,stash:2,stat:2,state:[2,8],std:22,stdout:22,step:[1,8,9,21,22,23],steppedsim:2,steputmr:2,still:[1,2],stochast:[9,22],stop:[9,23],store:15,str:[7,8,9,12,15,19,20,22,23],straight:4,stream:22,stuff:[1,2],style:1,sub:2,submodul:[1,5,6],subpackag:6,subsequ:[2,12],succe:[2,23],success:[8,13,15,21],success_observ:[8,12],success_reward:7,successfulli:16,successobserv:13,sudo:[1,2],suer:2,suffici:1,suggest:2,sum:15,summari:15,supersuit:2,support:8,sure:[2,22],surfac:5,symbol:2,system:[1,5],t1:[2,19],t:2,tab:2,take:[1,2],taken:[8,22],tan:13,task_id:19,tensor:11,tensorboard:[3,23],tensorboard_writ:[7,8],tensorboardwrit:23,tensorboardx:15,termin:2,test:[1,2,5,9],text:2,textbox:2,than:2,thei:[1,2,4,22],them:[1,2,4,15,22],theta:[13,22],thi:[2,5,11,18,22],think:22,thirdparti:2,those:2,threshold:9,through:[2,4,20],throughout:3,time:[1,2],time_limit:[7,8],timelimit:7,timelimit_threshold:7,timelimitwrapp:23,timestep:[12,13,15,16,23],timestep_threshold:23,tip:1,tmp:2,tmp_config:2,todo:18,toggl:3,ton:2,tool:2,toolchain:2,top:2,torch:11,torch_lay:11,total:15,touch:2,toward:2,track:[3,12,14,15,16,17,18],train:[1,2,3,4,5,7,8,9],train_length:7,translat:2,tree:[],trial:7,trick:1,trigger:9,trivial:2,tupl:[8,12,15,16,20,22,23],tutori:2,tweak:3,two:[0,2,4],txt:[1,2],type:[2,8,12,15,18,19],typo:2,udpat:2,unabl:2,uncom:2,undefin:2,union:[20,22,23],uniqu:2,unit:[11,16],unless:2,unnecessarili:1,until:[18,20],up:[0,1],updat:[1,2,9],update_child_loc:9,us:[1,3,9,12,15,16,17,18,20,22],use_pedsim:2,user:22,usernam:2,usual:2,ut:[],util:[7,8,23],utmr:[2,7,8,10,12],utmrsrespons:[8,10,12,21],utmrsstepp:2,v3:22,valid:2,valu:[2,3,18,22],varaibl:2,vari:3,variabl:[1,2,9],variat:22,variou:3,vd:1,ve:2,vec_env:22,vecenv:22,vector:22,vector_displai:1,vectordisplai:1,vectormap:2,vel_i:13,vel_theta:13,vel_x:13,veloc:[13,16],velocity_control:[8,15],velocitycontrol:16,venv:1,verbos:9,veri:2,version:1,vertic:2,via:[5,13],video:[2,23],vim:2,virtualenv:[],visual:[3,8],vtheta:13,vx:13,vy:13,wa:[2,13],wai:[1,2,3],wait:2,wait_for_servic:2,wall:4,want:[2,22],warn:[2,9,22],wasn:2,we:[1,2,5,22],weak_out_of_zon:17,weight:[15,16,17,18],weird:2,welcom:2,well:[1,2,3,5,15,22],went:[],what:[0,1,2,4,20],when:[1,2,9,16,22,23],where:[1,2,9,22],whether:[9,22],which:[1,2,5,8,20],why:[0,2],window:1,wish:1,within:23,won:2,work:[1,2,5,18,22],worri:[],worth:2,would:2,wrap:[9,22,23],wrapper:[3,7,8,9,15,22],write:[2,22],writer:15,written:1,wrong:[],x:[13,22],xdg_runtime_dir:2,xhost:1,xml:2,y:[13,22],ye:2,yep:2,yet:2,you:[1,2,3,5,9,22,23],your:[1,3],zone:[10,14]},titles:["Welcome to Social Gym\u2019s documentation!","Basic Install","Full Install","What\u2019s new in Social Gym 2.0 from the original release?","How to make a new Scene","What is Social Gym","src","src package","src.environment package","src.environment.callbacks package","src.environment.environment_types package","src.environment.extractors package","src.environment.observations package","src.environment.observations.types package","src.environment.observations.types.manual_zone package","src.environment.rewards package","src.environment.rewards.types package","src.environment.rewards.types.manual_zone package","src.environment.rewards.wrappers package","src.environment.scenarios package","src.environment.scenarios.types package","src.environment.services package","src.environment.utils package","src.environment.wrappers package"],titleterms:{"0":3,"1":[1,2],"2":[1,2,3],"3":[1,2],"4":[1,2],"5":[1,2],"6":1,"7":1,"8":[],"case":[],"new":[3,4],"static":2,The:2,agent_in_zon:14,agent_zone_current_ord:14,agent_zone_priority_ord:14,agents_goal_dist:13,agents_heading_direct:13,agents_others_dist:13,agents_pos:13,agents_preferred_veloc:13,agents_veloc:13,ar:1,basic:[1,2],branch:1,buckl:2,build:[1,2],callback:9,chang:[],checkout:1,clion:2,collis:16,collision_episode_end:23,collision_observ:13,common_observ:12,common_reward:15,common_scenario:19,config:1,config_run:7,contain:2,content:[7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],copi:1,creat:1,cycl:20,depend:[1,2],displai:1,docker:2,document:0,don:1,enforced_ord:17,entropy_episode_end:23,environ:[1,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],environment_typ:10,evaluate_polici:22,eventu:1,everyth:2,existence_penalti:16,extractor:11,faq:2,featur:3,file:1,folder:2,follow:2,from:[2,3],full:2,fun:1,goal_dist:16,goal_distance_chang:16,graph:2,graph_nav:20,graph_navig:2,guid:[1,2],gym:[0,1,2,3,5],have:1,here:[],host:[],how:4,i:2,id:2,indic:0,instal:[1,2],let:1,linear_weight_schedul:18,lstm_agent_ob:11,mai:[],main:1,make:4,manual:20,manual_zon:[10,14,17],modul:[7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],multi_agent_optuna:7,multirobot:2,navig:2,necessari:[],new_scenario_wrapp:23,note:[],observ:[12,13,14],open:2,origin:3,other_agent_observ:13,others_heading_direct:13,others_pos:13,others_veloc:13,packag:[7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],part:2,preferred_veloc:16,project:2,pycharm:2,python:[1,2],realli:1,releas:3,remot:[],repeat:[],requir:[1,2],reward:[15,16,17,18],reward_stripp:23,right:1,ro:2,root:2,ros_social_gym:8,run:[1,2],runner:1,s:[0,1,3,5],scenario:[19,20],scene:4,separ:2,servic:21,set:2,setup:2,sim:2,social:[0,1,2,3,5],social_norm_cross:16,social_norm_overtak:16,social_norm_pass:16,some:[],specif:2,src:[6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],ssh:2,stack:5,step:2,submodul:[2,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23],subpackag:[7,8,12,13,15,16,19],success:16,success_observ:13,t:1,tabl:0,tensorboard_writ:23,term:[],thei:[],thi:1,time:[],time_limit:23,two:1,type:[13,14,16,17,20],up:2,us:2,ut:2,ut_multirobot_sim:2,util:22,utmr:21,vector:1,velocity_control:16,version:[],via:2,virtualenv:1,we:[],welcom:0,what:[3,5],why:1,window:2,work:[],worri:1,wrapper:[18,23],you:[],your:2}})