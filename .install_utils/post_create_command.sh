#!/bin/bash

# Setup permissions
# USER="$(id -u -n)"
sudo chown -R $USERNAME /home/$USERNAME/ws/

# Install dependencies
source /opt/ros/humble/setup.bash
rosdep update &&
    rosdep install --from-paths /home/$USERNAME/ws/src --ignore-src -y

# Setup race_stack
bash ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh || echo "Failed to setup f110_sim"
bash ~/ws/src/race_stack/.install_utils/gb_opt_setup.sh || echo "Failed to setup gb_opt"

# Apply Joystick patch
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*

# setup f1tenth_gym
cd ~/ws &&
    colcon build --packages-up-to f110_gym --base-paths ~/ws \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic --cmake-clean-cache

echo "source ~/ws/src/race_stack/.install_utils/bashrc/.bashrc_git" >>/home/$USERNAME/.bashrc &&
    echo "source ~/ws/src/race_stack/.install_utils/bashrc/.bashrc_hmcar" >>/home/$USERNAME/.bashrc

cd ~/ws &&
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# colcon build --symlink-install
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
