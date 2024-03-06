cmake_minimum_required(VERSION 3.0.2)
project(lap_analyser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  f110_msgs
  tf
  tf_conversions
  visualization_msgs 
)

# Find Eigen3
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs f110_msgs tf visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(scan_align_analyzer_cppnode src/lap_analyser/ScanAlignAnalyzer.cpp)
target_link_libraries(scan_align_analyzer_cppnode ${catkin_LIBRARIES})

