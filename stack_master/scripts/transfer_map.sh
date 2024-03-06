#!/bin/bash

# This script transfers the YAML files from the given map in LOCAL map_editor to the
# REMOTE (car) overtaking_sector_tuner and the sector_tuner.

cd "${0%/*}"
cd ../..
# This ensures the current working directory is race_stack

# Process ROS_MASTER_URI
IFS=":"
read -ra ARR <<< "$ROS_MASTER_URI"
ROS_MASTER=$(echo "${ARR[1]}" | awk '{ print substr( $0, 3 ) }')    # remove '//'
echo "ROS master at: $ROS_MASTER"

REMOTE_MAPS_PATH="/home/race_crew/catkin_ws/src/race_stack/stack_master/maps"
LOCAL_MAP_EDITOR_PATH="f110_utils/nodes/map_editor/maps"

if [ $# -gt 0 ]; then
    echo "Listing contents of map $1 (local)"
    ls $LOCAL_MAP_EDITOR_PATH/$1
    echo -e "\n"

    # Uncomment if needed.
    echo "Copying map to remote..."
    ssh race_crew@$ROS_MASTER "mkdir -p $REMOTE_MAPS_PATH/$1"
    scp $LOCAL_MAP_EDITOR_PATH/$1/* race_crew@$ROS_MASTER:$REMOTE_MAPS_PATH/$1/

    echo "Copying Overtaking Sector Tuner files to remote..."
    scp $LOCAL_MAP_EDITOR_PATH/$1/ot_sectors.yaml race_crew@$ROS_MASTER:/home/race_crew/catkin_ws/src/race_stack/f110_utils/nodes/overtaking_sector_tuner/cfg/ot_sectors.yaml
    # cp stack_master/maps/$1/ot_sectors.yaml f110_utils/nodes/overtaking_sector_tuner/cfg/ot_sectors.yaml
    echo -e "\n"

    echo "Copying Sector Tuner files to remote..."
    scp $LOCAL_MAP_EDITOR_PATH/$1/speed_scaling.yaml race_crew@$ROS_MASTER:/home/race_crew/catkin_ws/src/race_stack/f110_utils/nodes/sector_tuner/cfg/speed_scaling.yaml
    # cp stack_master/maps/$1/speed_scaling.yaml f110_utils/nodes/sector_tuner/cfg/speed_scaling.yaml
    echo -e "\n"

    echo "Building on remote..."
    ssh race_crew@$ROS_MASTER "\
    cd /home/race_crew/catkin_ws && \
    catkin clean sector_tuner overtaking_sector_tuner &&  \
    catkin build --continue sector_tuner overtaking_sector_tuner &&  \
    source /home/race_crew/catkin_ws/devel/setup.bash; \
    "
    echo -e "\n"
    echo "Done! Please remember to source in your other terminals."
else
    echo "Please specify a map name for this to work."
    echo "A list of local maps in the map_editor folder:"
    ls -la f110_utils/nodes/map_editor/maps
    echo -e "\n#######################################################################\n"
    echo "A list of remote maps:"
    ssh race_crew@$ROS_MASTER ls -la /home/race_crew/catkin_ws/src/race_stack/stack_master/maps
fi