#!/bin/bash
# usage: run_zenoh.sh [OPP_IP]
# With OPP_IP the local router peers with the opponent car's router, so that
# car_to_car_sync can subscribe to the opponent's domain through it.
source /opt/ros/humble/setup.bash
source $HOME/ws/install/setup.bash
if [ -n "$1" ]; then
    export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/'$1':7447"]'
fi
ros2 run rmw_zenoh_cpp rmw_zenohd
