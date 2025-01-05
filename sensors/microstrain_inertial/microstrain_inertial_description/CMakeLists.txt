cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_description)

find_package(ament_cmake REQUIRED)

# Install the launch and config files
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()