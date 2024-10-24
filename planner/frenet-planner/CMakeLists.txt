cmake_minimum_required(VERSION 3.10)
project(frenet_planner)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall -DQT_NO_VERSION_TAGGING")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(catkin REQUIRED COMPONENTS 
	roscpp
        f110_msgs
        frenet_conversion
        nav_msgs
        geometry_msgs
        visualization_msgs
        tf
)

find_package(Eigen3)

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN_INCLUDE_DIR ${EIGEN_INCLUDE_DIR})
endif()

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp f110_msgs nav_msgs geometry_msgs visualization_msgs tf 
                  frenet_conversion
   LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
        ${catkin_INCLUDE_DIRS} 
        ${CUSTOM_EIGEN_INCLUDE_DIR}
        ${Threads_INCLUDE_DIRS})

add_executable(frenet_planner_node
               src/frenet_planner_node.cpp
               src/QuarticPolynomial.cpp
               src/QuinticPolynomial.cpp
               src/CubicSpline1D.cpp
               src/CubicSpline2D.cpp
               src/FrenetOptimalTrajectory.cpp
               src/FrenetPath.cpp)

target_link_libraries(frenet_planner_node 
                ${catkin_LIBRARIES} 
                Eigen3::Eigen 
                Qt5::Widgets 
                Qt5::Gui 
                Threads::Threads)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)