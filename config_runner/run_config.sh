#!/bin/bash

xhost +

DIR=$(pwd)

#sudo docker rm $1

docker run -d --name $1 -w /home/rosdev/social_gym/ --gpus all --privileged \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=unix$DISPLAY \
-v ${DIR}/data:/home/rosdev/social_gym/data \
-v ${DIR}/config_runner/configs:/home/rosdev/social_gym/config_runner/configs \
-v ${DIR}/submodules/amrl_maps/envs:/home/rosdev/social_gym/submodules/amrl_maps/envs \
-v ${DIR}/submodules/ut_multirobot_sim/maps:/home/rosdev/social_gym/submodules/ut_multirobot_sim/maps \
--network host \
-v ${DIR}/src:/home/rosdev/social_gym/src \
social_gym_config_runner:1.0 \
bash -c \
"(source config_runner/set_paths.sh && roscore &) && sleep 4 && (export DOCKER=false && source config_runner/set_paths.sh && export PYTHONPATH=\$PYTHONPATH:/home/rosdev/social_gym && pip show supersuit && pip show pettingzoo && pip show stable-baselines3 && python --version && python src/config_run.py -c ${2})"

