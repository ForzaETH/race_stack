#!/bin/bash

# Stop at error
set -e

# Setup user permissions
sudo chown -R race_crew:race_crew /ws/

if [ -z "$(find /ws/src -mindepth 1 -maxdepth 1 -type d -not -name race_stack -print -quit)" ]; then
	echo "No external repositories found in /ws/src. Run 'make deps' in the race_stack folder on the host, then rebuild the container."
	exit 1
fi

sudo apt-get update

# Install ROS 2 dependencies (race_stack and additional cloned repos)
rosdep update
rosdep install --from-paths /ws/src --ignore-src -y

# f110_gym is a plain Python library (ignored by colcon), install it editable so changes apply directly
pip3 install --user --no-deps -e /ws/src/f1tenth_gym

# Copy the sample maps into MAPS_DIR
for map in /ws/src/race_stack/sample_maps/*/; do
	name=$(basename "$map")
	[ -e "${MAPS_DIR:?MAPS_DIR is not set}/$name" ] || cp -r "$map" "$MAPS_DIR/$name"
done

# Set joystick permissions
sudo chmod 666 /dev/input/js0 2>/dev/null || true
sudo chmod 666 /dev/input/event* 2>/dev/null || true

# Build workspace
cd /ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
