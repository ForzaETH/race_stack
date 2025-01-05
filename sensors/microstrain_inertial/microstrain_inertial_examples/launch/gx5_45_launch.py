import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_GX5_45_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_45', 'gx5_45.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'gx5_45', 'display.rviz')

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _GX5_45_PARAMS_FILE,
        'namespace': '/',
      }.items()
    ),

    # Publish a static transform for where the GX5-45 is mounted on base_link.
    # Unless the GX5-45 is mounted directly at base_link, you should change this to be accurate to your setup.
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=[
          "--x", "0.0",
          "--y", "0.0",
          "--z", "0.0",
          "--roll", "0.0",
          "--pitch", "0.0",
          "--yaw", "0.0",
          "--frame-id", "base_link",
          "--child-frame-id", "gx5_45_link"
        ]
    ),

    # Run rviz to view the state of the application
    Node(
      package='rviz2',
      executable='rviz2',
      output='log',
      arguments=[
        '-d', _RVIZ_DISPLAY_FILE
      ]
    ),
  ])