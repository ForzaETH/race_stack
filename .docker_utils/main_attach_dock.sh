#! /bin/bash

# Script to launch the main docker instance for the pblf110 car when the container was already created
docker start forzaeth_racestack_ros2_humble
docker attach forzaeth_racestack_ros2_humble
