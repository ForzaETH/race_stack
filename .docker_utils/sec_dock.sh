#! /bin/bash

# Script to launch the main docker instance for the pblf110 car

docker exec --tty \
    --interactive \
    sim_x86_forzaeth_racestack_ros2_jazzy \
    /bin/bash
