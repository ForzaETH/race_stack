#!/bin/bash

# Stop at error
set -e

# Setup user permissions
sudo chown -R race_crew:race_crew /ws/

# Clone repositories into src
sudo apt-get update
vcs import /ws/src < /ws/src/race_stack/.install_utils/dependencies.repos

# Initialize submodules in all imported Git repositories. 
# Note that we avoid this on the race_stack, but for external repositories we do not handle it
while IFS= read -r -d '' git_dir; do
        git -C "$(dirname "$git_dir")" submodule update --init --recursive
done < <(find /ws/src -type d -name .git -print0)

# Import manifests shipped by cloned repositories. Their paths are relative to /ws.
while IFS= read -r repos_file; do
	vcs import /ws < "$repos_file"
done < <(find /ws/src -mindepth 2 -name '*.repos' \
	-not -path '/ws/src/race_stack/.install_utils/dependencies.repos' -print)

# Install ROS 2 dependencies (race_stack and additional cloned repos)
rosdep update
rosdep install --from-paths /ws/src --ignore-src -y

# Set joystick permissions 
sudo chmod 666 /dev/input/js0 2>/dev/null || true
sudo chmod 666 /dev/input/event* 2>/dev/null || true

# Build workspace
cd /ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
