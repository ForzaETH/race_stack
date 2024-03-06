#! /bin/bash

# extend .bashrc
cat /home/${USER}/catkin_ws/src/race_stack/.devcontainer/.install_utils/bashrc_ext >> ~/.bashrc

# source
source /opt/ros/noetic/setup.bash && source /home/${USER}/catkin_ws/devel/setup.bash

# build 
cd /home/${USER}/catkin_ws
catkin build
