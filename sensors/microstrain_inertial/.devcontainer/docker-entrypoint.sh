#!/bin/bash

set -e

# Source the ROS setup bash
source /opt/ros/${DOCKER_ROS_VERSION}/setup.bash

# Run roslaunch with whatever params were passed
exec ros2 launch microstrain_inertial_driver microstrain_launch.py $@
