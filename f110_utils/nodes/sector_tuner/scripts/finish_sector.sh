#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws 

#Clean and build sector tuner to force refresh of dynamic reconfigure
catkin clean sector_tuner
catkin build sector_tuner