#! /bin/bash

# Script to launch the main docker instance for the pblf110 car when the container was already created
docker start nuc_forzaeth_racestack_ros2
docker attach nuc_forzaeth_racestack_ros2
