#! /bin/bash

# extend .bashrc
cat /home/${USER}/catkin_ws/src/race_stack/.devcontainer/.install_utils/bashrc_ext >> ~/.bashrc

# source
source /opt/ros/noetic/setup.bash && source /home/${USER}/catkin_ws/devel/setup.bash

# install dependencies
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/ccma
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/forza_helpers
pip install ~/catkin_ws/src/race_stack/planner/graph_based_planner/src/GraphBasedPlanner
pip install ~/catkin_ws/src/race_stack/perception/norfair_tracker/src/norfair

# build 
cd /home/${USER}/catkin_ws
catkin build
