cmake_minimum_required(VERSION 3.10)
project(frenet_conversion)

find_package(catkin REQUIRED COMPONENTS 
        roscpp
        f110_msgs
        std_msgs
        message_generation
)

add_service_files(
        FILES
        Frenet2Glob.srv
        Frenet2GlobArr.srv
        Glob2Frenet.srv
        Glob2FrenetArr.srv
)

catkin_python_setup()

generate_messages()

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp f110_msgs
   LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
        ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/frenet_conversion.cc)

target_link_libraries(${PROJECT_NAME} 
                ${catkin_LIBRARIES})

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


catkin_install_python(PROGRAMS
  src/frenet_converter/frenet_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)