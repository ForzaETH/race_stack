#!/bin/bash

# This script transfers the YAML files from the given map in stack_master to the
# overtaking_sector_tuner and the sector_tuner.

cd "${0%/*}"
cd ../..
# This ensures the current working directory is race_stack

MAP_PATH="stack_master/maps"

if [ $# -gt 0 ]; then
    echo "Listing contents of map $1"
    ls $MAP_PATH/$1
    echo -e "\n"

    echo "Copying Overtaking Sector Tuner files..."
    cp $MAP_PATH/$1/ot_sectors.yaml f110_utils/nodes/overtaking_sector_tuner/cfg/ot_sectors.yaml
    echo -e "\n"

    echo "Copying Sector Tuner files..."
    cp $MAP_PATH/$1/speed_scaling.yaml f110_utils/nodes/sector_tuner/cfg/speed_scaling.yaml
    echo -e "\n"

    echo "Building ..."
    cd ../..;   # now we are in catkin_ws
    catkin clean sector_tuner overtaking_sector_tuner &&  \
    catkin build sector_tuner overtaking_sector_tuner &&  \
    source /home/race_crew/catkin_ws/devel/setup.bash;

    echo -e "\n"
    echo "Done! Please remember to source in your other terminals."
else
    echo "Please specify a map name for this to work."
    echo "A list of local maps in the stack_master folder:"
    ls -la $MAP_PATH
    echo -e "\n#######################################################################\n"
fi