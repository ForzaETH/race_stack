cmake_minimum_required(VERSION 3.0.2)
project(id_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  f110_msgs
)

catkin_python_setup()
