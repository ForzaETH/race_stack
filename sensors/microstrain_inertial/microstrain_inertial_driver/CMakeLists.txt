cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_driver)

# C++ 14 required
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Export compile commands by default (helpful for clang-tidy and autocomplete for certain IDEs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "" FORCE)

# Force some options for this module and the MIP SDK
set(BUILD_SHARED_LIBS_TEMP "${BUILD_SHARED_LIBS}")
set(BUILD_EXAMPLES_TEMP "${BUILD_EXAMPLES}")
set(BUILD_TESTING_TEMP "${BUILD_TESTING}")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_driver_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")
set(COMMON_SRC_DIR "${COMMON_DIR}/src")
set(COMMON_INC_DIR "${COMMON_DIR}/include/${COMMON_NAME}")
set(MIP_SDK_DIR "${COMMON_DIR}/mip_sdk")
set(MIP_SDK_SRC_DIR "${MIP_SDK_DIR}/src")

# Some helpful options for debugging
add_compile_options($<$<CONFIG:DEBUG>:-ggdb>)

# Try to find Git
find_package(Git)

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_SRC_DIR}")
  message(STATUS "Initializing ${COMMON_DIR} submodule. This should only happen once.")

  # Make sure we can find the git executable
  if(NOT Git_FOUND)
    message(FATAL_ERROR "Unable to initialize submodule because we could not find the git executable. Please clone this repo using 'git clone --recursive'")
  endif()

  # Initialize and update the submodule
  execute_process(
    WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} submodule update --recursive --init
  )
endif()

# Use Git to find the version
set(DEFAULT_DRIVER_GIT_VERSION "unknown")
if(NOT GIT_FOUND)
  message(STATUS "Unable to find git, will build with unknown version")
  set(DRIVER_GIT_VERSION ${DEFAULT_DRIVER_GIT_VERSION})
else()
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} describe --tags
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE DRIVER_GIT_VERSION_OUT
    ERROR_VARIABLE DRIVER_GIT_VERSION_ERR
    RESULT_VARIABLE DRIVER_GIT_VERSION_RET
  )
  if(NOT ${DRIVER_GIT_VERSION_RET} EQUAL 0)
    message(STATUS "Unable to determine version from Git, defaulting to version ${DEFAULT_DRIVER_GIT_VERSION}")
    set(DRIVER_GIT_VERSION ${DEFAULT_DRIVER_GIT_VERSION})
  else()
    set(DRIVER_GIT_VERSION ${DRIVER_GIT_VERSION_OUT})
    string(REGEX REPLACE "\n" "" DRIVER_GIT_VERSION "${DRIVER_GIT_VERSION}")
    message(STATUS "Microstrain Driver Version: ${DRIVER_GIT_VERSION}")
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-unused-variable -fno-builtin-memcpy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-builtin-memcpy")
endif()

# Include the MIP SDK source
add_subdirectory("${MIP_SDK_DIR}" EXCLUDE_FROM_ALL)
include_directories("${MIP_SDK_SRC_DIR}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(microstrain_inertial_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
ament_export_include_directories()
ament_export_dependencies(
    roscpp
    rosidl_default_runtime
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    nmea_msgs
    rtcm_msgs
    message_runtime
		microstrain_inertial_msgs
)

###########
## Build ##
###########

# Find some less ROS-y packages
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib/")
list(APPEND CMAKE_MODULE_PATH "/usr/lib/aarch64-linux-gnu/")
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

# We want to use the static library for libgeographic, but it doesn't seem to find it for us, so substitute it here
string(REGEX REPLACE "[.]so[^\\/\\s]*$" ".a" GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES})

# Manually specify the path to the static library
find_library(GEOGRAPHIC_STATIC_LIB 
  NAMES "libGeographic.a"
  PATHS 
    "/usr/lib"
    "/usr/lib/aarch64-linux-gnu"
    "/usr/lib/x86_64-linux-gnu"
)

# Check if the static library exists
if (EXISTS ${GEOGRAPHIC_STATIC_LIB})
  message(STATUS "Using static library: ${GEOGRAPHIC_STATIC_LIB}")
  set(GeographicLib_LIBRARIES ${GEOGRAPHIC_STATIC_LIB})
  include_directories("/usr/include/GeographicLib")
else()
  message(FATAL_ERROR "Static library libGeographic.a not found in /usr/lib/aarch64-linux-gnu/")
endif()

# Add the catkin includes
include_directories(
  include
  ${COMMON_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

ament_package()

set(COMMON_SRC_FILES
  ${COMMON_SRC_DIR}/subscribers.cpp
  ${COMMON_SRC_DIR}/publishers.cpp
  ${COMMON_SRC_DIR}/node_common.cpp
  ${COMMON_SRC_DIR}/services.cpp
  ${COMMON_SRC_DIR}/config.cpp

  ${COMMON_SRC_DIR}/utils/clock_bias_monitor.cpp

  ${COMMON_SRC_DIR}/utils/mappings/mip_mapping.cpp
  ${COMMON_SRC_DIR}/utils/mappings/mip_publisher_mapping.cpp

  ${COMMON_SRC_DIR}/utils/mip/ros_mip_device.cpp
  ${COMMON_SRC_DIR}/utils/mip/ros_mip_device_main.cpp
  ${COMMON_SRC_DIR}/utils/mip/ros_mip_device_aux.cpp
  ${COMMON_SRC_DIR}/utils/mip/ros_connection.cpp

)
set(COMMON_INC_FILES
  ${COMMON_INC_DIR}/subscribers.h
  ${COMMON_INC_DIR}/publishers.h
  ${COMMON_INC_DIR}/node_common.h
  ${COMMON_INC_DIR}/services.h
  ${COMMON_INC_DIR}/config.h
  ${COMMON_INC_DIR}/utils/ros_compat.h
  ${COMMON_INC_DIR}/utils/clock_bias_monitor.h

  ${COMMON_INC_DIR}/utils/mappings/mip_mapping.h
  ${COMMON_INC_DIR}/utils/mappings/mip_publisher_mapping.h

  ${COMMON_INC_DIR}/utils/mip/ros_mip_device.h
  ${COMMON_INC_DIR}/utils/mip/ros_mip_device_main.h
  ${COMMON_INC_DIR}/utils/mip/ros_mip_device_aux.h
  ${COMMON_INC_DIR}/utils/mip/ros_connection.h

)
set(COMMON_FILES ${COMMON_SRC_FILES} ${COMMON_INC_FILES})

set(AMENT_COMMON_DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  rtcm_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  microstrain_inertial_msgs
)

# Plain node
set(NODE_NAME ${PROJECT_NAME}_node)
set(NODE_SRC_FILES
  src/microstrain_inertial_driver.cpp
  src/microstrain_inertial_driver_node.cpp
)
set(NODE_INC_FILES
  include/${PROJECT_NAME}/microstrain_inertial_driver.h
)
set(NODE_FILES ${NODE_SRC_FILES} ${NODE_INC_FILES})
add_executable(${NODE_NAME} ${NODE_FILES} ${COMMON_FILES})
ament_target_dependencies(${NODE_NAME}
  ${AMENT_COMMON_DEPENDENCIES}
)

# Lifecycle node
set(LIFECYCLE_NAME ${PROJECT_NAME}_lifecycle_node)
set(LIFECYCLE_SRC_FILES
  src/microstrain_inertial_driver_lifecycle.cpp
  src/microstrain_inertial_driver_lifecycle_node.cpp
)
set(LIFECYCLE_INC_FILES
  include/${PROJECT_NAME}/microstrain_inertial_driver_lifecycle.h
)
set(LIFECYCLE_FILES ${LIFECYCLE_SRC_FILES} ${LIFECYCLE_INC_FILES})
add_executable(${LIFECYCLE_NAME} ${LIFECYCLE_FILES} ${COMMON_FILES})
target_compile_definitions(${LIFECYCLE_NAME} PUBLIC MICROSTRAIN_LIFECYCLE)
ament_target_dependencies(${LIFECYCLE_NAME}
  ${AMENT_COMMON_DEPENDENCIES}
)

# Annoying, but the ROS types don't match up with the MIP types for floats and doubles, so ignore those warnings for now
set_source_files_properties(${COMMON_SRC_DIR}/services.cpp PROPERTIES COMPILE_OPTIONS "-Wno-narrowing")

# Tell the code the version of the driver that is being build
add_definitions(-DMICROSTRAIN_DRIVER_VERSION="${DRIVER_GIT_VERSION}")

# Let the code know if it is being compiled with ROS1 or ROS2
if(DEFINED ENV{ROS_VERSION})
  add_definitions(-DMICROSTRAIN_ROS_VERSION=$ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

# Some ROS version specific defines
if(DEFINED ENV{ROS_DISTRO})
  if ("$ENV{ROS_DISTRO}" STREQUAL "rolling")
    add_definitions(-DMICROSTRAIN_ROLLING=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_definitions(-DMICROSTRAIN_HUMBLE=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "galactic")
    add_definitions(-DMICROSTRAIN_GALACTIC=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_definitions(-DMICROSTRAIN_FOXY=1)
  else()
    add_definitions(-DMICROSTRAIN_ROLLING=1)  # By default assume this is a newer ROS version than we support, so rolling define is the safest
  endif()
endif()

# Linking
target_link_libraries(${LIFECYCLE_NAME}
  mip
  ${GeographicLib_LIBRARIES}
)
target_link_libraries(${NODE_NAME}
  mip
  ${GeographicLib_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${LIFECYCLE_NAME}
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${NODE_NAME}
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${COMMON_DIR}/config
  DESTINATION share/${PROJECT_NAME}/${COMMON_NAME}
)

# Reset the forced options
set(BUILD_SHARED_LIBS "${BUILD_SHARED_LIBS_TEMP}" CACHE BOOL "${BUILD_SHARED_LIBS_TEMP}" FORCE)
set(BUILD_EXAMPLES "${BUILD_EXAMPLES_TEMP}" CACHE BOOL "${BUILD_EXAMPLES_TEMP}" FORCE)
set(BUILD_TESTING "${BUILD_TESTING_TEMP}" CACHE BOOL "${BUILD_TESTING_TEMP}" FORCE)