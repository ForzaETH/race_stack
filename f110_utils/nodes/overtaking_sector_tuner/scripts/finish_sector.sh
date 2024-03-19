#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws 

#Clean and build sector tuner to force refresh of dynamic reconfigure
catkin clean overtaking_sector_tuner
catkin build overtaking_sector_tuner