cmake_minimum_required(VERSION 3.0.2)
project(obstacle_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  f110_msgs
  dynamic_reconfigure
)

## Use this to call setup.py
catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/dyn_obs_publisher.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs f110_msgs dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
