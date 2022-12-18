#!/bin/bash

#set -o xtrace

if [ $# -eq 0 ]
  then
    echo "Please specify a map name as an argument eg './vector_display.sh tmp_map'"
    exit 0
fi


xhost +

DIR=$(pwd)

# For new maps, make their corresponding directory and vectormap file.
[ -d "$DIR/maps/$1" ]  || mkdir "$DIR/maps/$1"
[ -f "$DIR/maps/$1/$1.vectormap.txt" ] || touch "$DIR/maps/$1/$1.vectormap.txt"

touch "$DIR/maps/$1/$1.navigation.txt"

# Share the maps directory, and some other stuff for showing the display.
sudo docker rm -f VECTORDISPLAY && sudo docker run --name VECTORDISPLAY -w /root/vector_display/ \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY  \
-v ${DIR}/maps:/root/vector_display/maps/:rw \
vector_display:custom \
bash -c \
"./bin/vector_display --map=$*"

# 1. keep a back up of the map just in case something happens to the raw txt file.
# 2. There's a weird encoding issue (a phantom character) prepended to the file that causes the sim to break when ran
#    overwriting the file solves this issue.
cat "$DIR/maps/$1/$1.vectormap.txt" > "$DIR/maps/$1/$1.vectormap.backup.txt"
cat "$DIR/maps/$1/$1.vectormap.backup.txt" > "$DIR/maps/$1/$1.vectormap.txt"

# Convert the raw txt file into a json format for the sim.
python utils/vectormap_txt_to_json.py -i "$DIR/maps/$1/$1.vectormap.txt" -o "$DIR/maps/$1/$1.vectormap.json"
python utils/vectormap_json_to_pedsim.py -i "$DIR/maps/$1/$1.vectormap.json" -o "$DIR/maps/$1/scene.xml"

# Update the maps folder in the submodules directory (where the maps are actually pulled for the sim)
rm "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.vectormap.txt"
rm "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.vectormap.json"
rm "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.navigation.json"
mkdir "$DIR/../../submodules/ut_multirobot_sim/maps/$1"
cp "$DIR/maps/$1/$1.vectormap.txt" "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.vectormap.txt"
cp "$DIR/maps/$1/$1.vectormap.json" "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.vectormap.json"
cp "$DIR/maps/$1/$1.navigation.json" "$DIR/../../submodules/ut_multirobot_sim/maps/$1/$1.navigation.json"

rm "$DIR/../../submodules/amrl_maps/$1/$1.vectormap.txt"
rm "$DIR/../../submodules/amrl_maps/$1/$1.vectormap.json"
rm "$DIR/../../submodules/amrl_maps/$1/$1.navigation.json"
mkdir "$DIR/../../submodules/amrl_maps/$1"
cp "$DIR/maps/$1/$1.vectormap.txt" "$DIR/../../submodules/amrl_maps/$1/$1.vectormap.txt"
cp "$DIR/maps/$1/$1.vectormap.json" "$DIR/../../submodules/amrl_maps/$1/$1.vectormap.json"
cp "$DIR/maps/$1/$1.navigation.json" "$DIR/../../submodules/amrl_maps/$1/$1.navigation.json"
set +o xtrace
