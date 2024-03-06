#!/bin/bash

echo "Go to docker folder"
cd /home/icra_crew/catkin_ws/src/icra_f110_stack/noetic_docker
echo "Build docker..."
docker build -t "ros-docker-f110" .
echo "Run planner..."
docker run -it --network=host --mount source="/home/icra_crew/catkin_ws/src/icra_f110_stack/stack_master/maps",type=bind,target=/maps ros-docker-f110:latest roslaunch f110_planner global_planner.launch
