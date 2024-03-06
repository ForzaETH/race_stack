cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  roslint
  serial
  std_msgs
  sensor_msgs
  vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nodelet
    pluginlib
    std_msgs
    sensor_msgs
    vesc_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(vesc_driver_node src/vesc_driver_node.cpp
                                src/vesc_driver.cpp
                                src/vesc_interface.cpp
                                src/vesc_packet.cpp
                                src/vesc_packet_factory.cpp)
add_dependencies(vesc_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_driver_node
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(vesc_driver_nodelet src/vesc_driver_nodelet.cpp
                                src/vesc_driver.cpp
                                src/vesc_interface.cpp
                                src/vesc_packet.cpp
                                src/vesc_packet_factory.cpp)
add_dependencies(vesc_driver_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_driver_nodelet
  ${catkin_LIBRARIES}
)

file(GLOB_RECURSE ${PROJECT_NAME}_CPP_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  . *.cpp *.h)
list(REMOVE_ITEM ${PROJECT_NAME}_CPP_FILES
  include/${PROJECT_NAME}/crc.h)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp(${${PROJECT_NAME}_CPP_FILES})

#############
## Install ##
#############

install(TARGETS vesc_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS vesc_driver_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES vesc_driver_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
