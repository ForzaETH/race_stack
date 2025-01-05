import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, SetRemap

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7', 'cv7.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7', 'display.rviz')

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _CV7_PARAMS_FILE,
        'namespace': '/',
      }.items()
    ),

    # In this example we have no way to publish an actual map transform, so just publish a static one so we can display data on rviz
    # If integrating into an existing system, this should be replaced with a navigation solution
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0",
          "--y", "0",
          "--z", "100",
          "--roll", "0",
          "--pitch", "0",
          "--yaw", "0",
          "--frame-id", "map",
          "--child-frame-id", "base_link"
        ]
    ),

    # Publish a static transform for where the CV7 is mounted on base_link.
    # Unless the CV7 is mounted exactly at base_link, you should change this to be accurate to your setup
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
          "--child-frame-id", "cv7_link"
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