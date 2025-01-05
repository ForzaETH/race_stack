# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_driver'
_DEFAULT_PARAMS_FILE = os.path.join(
  get_package_share_directory(_PACKAGE_NAME),
  'microstrain_inertial_driver_common',
  'config',
  'params.yml'
)
_GX5_25_PARAMS_FILE = os.path.join(
    get_package_share_directory('microstrain_inertial_examples'),
    'config',
    'gx5_25',
    'gx5_25.yml'
)

def generate_launch_description():
  car_name = os.getenv('F1TENTH_CAR_NAME', '')
  frame_id = 'gx5_25_link'
  mount_frame_id = 'base_link'
  gx5_namespace = 'gx5'

  if len(car_name) != 0:
    gx5_namespace = car_name + '/' + gx5_namespace
    mount_frame_id = mount_frame_id + '_' + car_name
    frame_id = frame_id + '_' + car_name
  nodename = f"{_PACKAGE_NAME}"
  # Declare arguments with default values
  launch_description = []
  launch_description.append(DeclareLaunchArgument('namespace',   default_value=gx5_namespace,                description='Namespace to use when launching the nodes in this launch file'))
  launch_description.append(DeclareLaunchArgument('node_name',   default_value=nodename,      description='Name to give the Microstrain Inertial Driver node'))
  launch_description.append(DeclareLaunchArgument('debug',       default_value='false',            description='Whether or not to log debug information.'))

  # Pass an environment variable to the node to determine if it is in debug or not
  launch_description.append(SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', value=LaunchConfiguration('debug')))


  # ****************************************************************** 
  # Microstrain sensor node 
  # ****************************************************************** 
  microstrain_node = Node(
    package    = _PACKAGE_NAME,
    executable = "microstrain_inertial_driver_node",
    name       = LaunchConfiguration('node_name'),
    namespace  = LaunchConfiguration('namespace'),
    parameters = [
      # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
      yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),
      yaml.safe_load(open(_GX5_25_PARAMS_FILE, 'r')),

      # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
      # LaunchConfiguration('params_file'),
      {
          'frame_id': frame_id,  # Append car_name to frame_id
          'mount_frame_id': mount_frame_id  # Append car_name to mount_frame_id
      },
    ]
  )

  launch_description.append(microstrain_node)
  return LaunchDescription(launch_description)
  

 
 
