cmake_minimum_required(VERSION 3.10)
project(frenet_odom_republisher)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
        frenet_conversion
        nav_msgs
        f110_msgs
        tf
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp nav_msgs f110_msgs frenet_conversion tf
   LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
        ${catkin_INCLUDE_DIRS} 
        ${CUSTOM_EIGEN_INCLUDE_DIR}
        ${Threads_INCLUDE_DIRS})

add_executable(frenet_odom_republisher_node
               src/frenet_odom_republisher_node.cc
               )

target_link_libraries(frenet_odom_republisher_node 
                ${catkin_LIBRARIES} 
                )

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)