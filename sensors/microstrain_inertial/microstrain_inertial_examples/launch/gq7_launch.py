import os
import math

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_GQ7_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gq7', 'gq7.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gq7', 'display.rviz')

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _GQ7_PARAMS_FILE,
        'namespace': '/',
      }.items()
    ),

    # Publish a static transform for where the GQ7 is mounted on base_link.
    # Unless the GQ7 is mounted exactly one meter above base_link, you should change this to be accurate to your setup
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0",
          "--y", "0",
          "--z", "0",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "base_link",
          "--child-frame-id", "gq7_link"
        ]
    ),

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])