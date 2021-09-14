#!/bin/bash

xhost +

DIR=$(pwd)

sudo docker run -d --name PPO -w /root/social_gym/ \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY  \
-v ${DIR}/docker/data:/root/social_gym/data/ \
social_gym:1.0 \
bash -c \
'(roscore &) && (export DOCKER=true && source scripts/set_paths.sh && python src/train_ppo.py 1)'
