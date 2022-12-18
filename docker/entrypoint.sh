#!/bin/bash

chown -R rosdev:sudo /home/rosdev/
source /home/rosdev/set_paths.sh
cd /home/rosdev/social_gym
export DISPLAY=:0
exec "$@"