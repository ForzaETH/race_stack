cmake_minimum_required(VERSION 2.8.3)
project(vesc_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  VescState.msg
  VescStateStamped.msg
  VescImu.msg
  VescImuStamped.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
