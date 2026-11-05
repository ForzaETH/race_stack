#!/bin/bash

# Stop at error
set -e

# Clone or update the external repositories (no-op for repos already at the pinned commit)
vcs import --recursive /ws/src < /ws/src/race_stack/.install_utils/dependencies.repos

# Install dependencies (env_keep in sudoers passes DEBIAN_FRONTEND through rosdep's sudo apt-get)
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
rosdep update
# python-transforms3d-pip was removed from rosdistro (transforms3d is installed via pip)
rosdep install --from-paths /ws/src --ignore-src -y --skip-keys "python-transforms3d-pip"

# Copy the sample maps into MAPS_DIR, keeping any map that is already there
for map in /ws/src/race_stack/sample_maps/*/; do
	name=$(basename "$map")
	[ -e "${MAPS_DIR:?MAPS_DIR is not set}/$name" ] || cp -r "$map" "$MAPS_DIR/$name"
done

# Apply Joystick patch
sudo chmod 666 /dev/input/js0 2>/dev/null || true
sudo chmod 666 /dev/input/event* 2>/dev/null || true

# Build workspace
cd /ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
