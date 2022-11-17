#!/bin/bash

xhost +

DIR=$(pwd)

#sudo docker rm $1

docker run -d --name $1 -w /home/rosdev/social_gym/ --gpus all \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY  \
-v ${DIR}/data:/home/rosdev/social_gym/data \
-v ${DIR}/config_runner/configs:/home/rosdev/social_gym/config_runner/configs \
-v ${DIR}/src:/home/rosdev/social_gym/src \
social_gym_config_runner:1.0 \
bash -c \
"(source config_runner/set_paths.sh && roscore &) && (export DOCKER=false && source config_runner/set_paths.sh && export PYTHONPATH=\$PYTHONPATH:/home/rosdev/social_gym && python src/config_run.py -c ${2})"

