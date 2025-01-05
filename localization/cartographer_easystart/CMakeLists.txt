cmake_minimum_required(VERSION 3.5)
project(cartographer_easystart)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Add C++ executable
add_executable(state_estimator src/state_estimator.cpp)

ament_target_dependencies(state_estimator
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_eigen
  Eigen3
)

install(TARGETS state_estimator
  DESTINATION lib/${PROJECT_NAME})

# Install Python script
install(
  PROGRAMS
    scripts/initialize.py scripts/kf.py 
  DESTINATION lib/${PROJECT_NAME}
)
