# f1tenth_base_ros2 Setup Instructions

## Add the Following to Your `.bashrc`

To streamline the usage of ROS2 and manage multi-agent settings efficiently, append the following lines to your `.bashrc` file. This will set environment variables, define aliases, and add a custom function for map saving.

```bash
# Set F1TENTH Car Name
export F1TENTH_CAR_NAME='hmcar1'

############################ ROS2 ##############################
# Aliases for building ROS2 packages
alias colb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias colbb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select '

# Multi-Agent Mode Check
alias joy='ros2 launch joy joy-launch.py'
alias lidar='ros2 launch urg_node urg_node_launch.py'
alias vesc='ros2 launch vesc vesc.launch.py'
alias imu='ros2 launch microstrain_inertial_examples gx5_25_launch.py'
alias cartomapping='ros2 launch cartographer_easystart mapping_2d_launch.py'
alias cartolocal='ros2 launch cartographer_easystart localization_2d_launch.py'

# Map Saving Aliases
alias mapsave='ros2 run nav2_map_server map_saver_cli --ros-args -r map:=/hmcar1/map'

# Function to Save Cartographer Maps
cartomapsave() {
  if [ -z "$1" ]; then
    echo "Usage: cartomapsave <filename_without_extension>"
    return 1
  fi
  FILENAME="/home/orin_nx/localization/src/cartographer_easystart/maps/$1.pbstream"
  echo "Saving map to: $FILENAME"  # Debugging line to check the filename
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: \"$FILENAME\"}"
}
