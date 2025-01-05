import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_INS_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7_ins', 'cv7_ins.yml')
_RVIZ_DISPLAY_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7_ins', 'display.rviz')

# Frame ID for our fake sensor
_FAKE_SENSOR_FRAME_ID = 'fake_sensor'

def generate_launch_description():
  return LaunchDescription([
    # Microstrain node
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
      launch_arguments={
        'configure': 'true',
        'activate': 'true',
        'params_file': _CV7_INS_PARAMS_FILE,
        'namespace': '/',
      }.items()
    ),
    
    # Publish mock information from a fake sensor to the CV7-INS
    ExecuteProcess(
      cmd=[
        "ros2", "topic", "pub", "/ext/llh_position", "sensor_msgs/msg/NavSatFix",
          f'''
          header:
            stamp: 'now'
            frame_id: '{_FAKE_SENSOR_FRAME_ID}'
          status:
            status: 0
            service: 0
          latitude: 44.4373567456773
          longitude: -73.10625770609381
          altitude: 126.0
          position_covariance:
          - 0.78
          - 0.0
          - 0.0
          - 0.0
          - 0.78
          - 0.0
          - 0.0
          - 0.0
          - 0.78
          position_covariance_type: 1
          '''
      ],
    ),
    ExecuteProcess(
      cmd=[
        "ros2", "topic", "pub", "/ext/velocity_enu", "geometry_msgs/msg/TwistWithCovarianceStamped",
          f'''
            header:
              stamp: 'now'
              frame_id: '{_FAKE_SENSOR_FRAME_ID}'
            twist:
              twist:
                linear:
                  x: 0.0
                  y: 0.0
                  z: 0.0
                angular:
                  x: 0.0
                  y: 0.0
                  z: 0.0
              covariance:
              - 0.01
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.01
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.01
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
              - 0.0
          '''
      ],
    ),

    # Publish a static transform for where the our fake sensor is mounted on base_link.
    # You should replace this with actual transforms for where your aiding sensors are
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
          "--child-frame-id", _FAKE_SENSOR_FRAME_ID
        ]
    ),

    # Publish a static transform for where the CV7-INS is mounted on base_link.
    # Unless the CV7-INS is mounted exactly at base_link, you should change this to be accurate to your setup
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
          "--child-frame-id", "cv7_ins_link"
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