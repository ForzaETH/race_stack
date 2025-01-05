cmake_minimum_required(VERSION 3.5)
project(urg_node)

find_package(ament_cmake REQUIRED)

if(WIN32)
  message(WARNING "${PROJECT_NAME} is currently not supported on Windows")
  ament_package()
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# we currently still rely on boost for its `crc` component
find_package(Boost REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(laser_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urg_c REQUIRED)
find_package(urg_node_msgs REQUIRED)

add_library(urg_node SHARED
  src/urg_c_wrapper.cpp
  src/urg_node.cpp
)
target_include_directories(urg_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(urg_node
  diagnostic_updater
  laser_proc
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  urg_c
  urg_node_msgs
  Boost
)
rclcpp_components_register_node(urg_node
  PLUGIN "urg_node::UrgNode"
  EXECUTABLE urg_node_driver)

add_executable(getID src/get_id.cpp)
target_link_libraries(getID urg_node)

install(TARGETS urg_node getID
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/set_urg_ip.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(urg_node_msgs)
ament_export_dependencies(
  diagnostic_updater
  laser_proc
  rclcpp
  sensor_msgs
  std_srvs
  urg_c
)
ament_export_include_directories(include)
ament_export_libraries(urg_node)
ament_package()
