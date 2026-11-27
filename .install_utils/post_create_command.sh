#!/bin/bash

# Setup permissions
# USER="$(id -u -n)"
sudo chown -R $USER:$USER ~/ws/

# Install dependencies
rosdep update &&
    rosdep install --from-paths /home/$USERNAME/ws/src --ignore-src -y \
        --skip-keys python-transforms3d-pip

# Setup race_stack
bash ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh || echo "Failed to setup f110_sim"
bash ~/ws/src/race_stack/.install_utils/gb_opt_setup.sh || echo "Failed to setup gb_opt"

# Copy the sample maps into maps (gitignored), keeping any map that is already there
mkdir -p "${MAPS_DIR:?MAPS_DIR is not set, see docker-compose.yaml}"
for map in ~/ws/src/race_stack/sample_maps/*/; do
    [ -e "$MAPS_DIR/$(basename "$map")" ] || cp -r "$map" "$MAPS_DIR/"
done
touch "$MAPS_DIR/COLCON_IGNORE"

# Apply Joystick patch
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*

# setup f1tenth_gym
cd ~/ws &&
    colcon build --packages-up-to f110_gym --base-paths ~/ws \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic --cmake-clean-cache

cd ~/ws/src/race_stack &&
    ./.vscode/build.sh Release
