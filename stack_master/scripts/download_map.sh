#!/bin/bash

# This script downloads a map from the car to local map_editor folder

cd "${0%/*}"
cd ../..
# This ensures the current working directory is race_stack

# Process ROS_MASTER_URI
IFS=":"
read -ra ARR <<< "$ROS_MASTER_URI"
ROS_MASTER=$(echo "${ARR[1]}" | awk '{ print substr( $0, 3 ) }')    # remove '//'
echo "ROS master at: $ROS_MASTER"

REMOTE_MAP_EDITOR_PATH="/home/race_crew/catkin_ws/src/race_stack/f110_utils/nodes/map_editor/maps"
LOCAL_MAP_EDITOR_PATH="f110_utils/nodes/map_editor/maps"

if [ $# -gt 0 ]; then
    echo "Listing contents of map $1 (remote)"
    ssh race_crew@$ROS_MASTER ls -la $REMOTE_MAP_EDITOR_PATH/$1
    echo -e "\n"

    echo "Copying map from remote to local map_editor folder..."
    mkdir -p $LOCAL_MAP_EDITOR_PATH/$1
    scp race_crew@$ROS_MASTER:$REMOTE_MAP_EDITOR_PATH/$1/* $LOCAL_MAP_EDITOR_PATH/$1/

    echo "Done! Copied to $LOCAL_MAP_EDITOR_PATH/$1"
else
    echo "Please specify a map name for this to work."
    echo "A list of remote maps in the map_editor folder:"
    ssh race_crew@$ROS_MASTER ls -la $REMOTE_MAP_EDITOR_PATH
fi