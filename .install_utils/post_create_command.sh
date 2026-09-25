#!/bin/bash

# Stop at error
set -e

# Setup user permissions
sudo chown -R race_crew:race_crew /ws/

# Clone external repositories into /ws/src (no submodules)
vcs import /ws/src < /ws/src/race_stack/.install_utils/dependencies.repos

# Install ROS 2 dependencies (race_stack and additional cloned repos)
rosdep update
rosdep install --from-paths /ws/src --ignore-src -y

# Set joystick permissions 
sudo chmod 666 /dev/input/js0 2>/dev/null || true
sudo chmod 666 /dev/input/event* 2>/dev/null || true

# Build workspace
cd /ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
