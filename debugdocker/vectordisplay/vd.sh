#!/bin/bash

#set -o xtrace

xhost +

# Share the maps directory, and some other stuff for showing the display.
sudo docker rm -f VECTORDISPLAY && sudo docker run --name VECTORDISPLAY -w /root/vector_display/ \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY  \
-v $1:/root/vector_display/maps/:rw \
vector_display:custom \
bash -c \
"./bin/vector_display --map=$2 $3"
#set +o xtrace
