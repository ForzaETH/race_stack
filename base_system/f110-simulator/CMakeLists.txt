cmake_minimum_required(VERSION 3.1)

project(f1tenth_simulator)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3)

######################################
# Compile the library >
######################################

# Add includes
include_directories(include)
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})

# Install the library to CMAKE_INSTALL_PREFIX
# which defaults to /usr/local
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

######################################
# < End compile the library
######################################

######################################
# ROS >
######################################

# Only run this code in the catkin environment
if(DEFINED CATKIN_DEVEL_PREFIX)
  find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    ackermann_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs

    interactive_markers
    visualization_msgs

    std_msgs
    message_generation
    )

  generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    )




  # Include catkin_libraries
  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS message_runtime
  )
  set(LIBS ${LIBS} ${catkin_LIBRARIES})
  include_directories(include ${catkin_INCLUDE_DIRS})


  

  # Add the nodes
  file(GLOB NODE_SRC_FILES node/*.cpp)
  foreach(_node_file ${NODE_SRC_FILES})
      get_filename_component(_node_name ${_node_file} NAME_WE)
      add_executable(${_node_name} ${_node_file})
      target_link_libraries(${_node_name} ${LIBS})
      add_dependencies(${_node_name} f1tenth_simulator_generate_messages_cpp)
  endforeach()

  # Install the library
  install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()

######################################
# < End ROS
######################################
