import os
import yaml
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    # Declare arguments
    launch_description = [
        DeclareLaunchArgument(
            'config_file',
            default_value=_GX5_25_PARAMS_FILE,
            description='Path to the configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='gx5',
            description='Namespace to use when launching the nodes in this launch file'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value=_PACKAGE_NAME,
            description='Name to give the Microstrain Inertial Driver node'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Whether or not to log debug information.'
        )
    ]

    # Use OpaqueFunction to handle dynamic parameter evaluation
    def launch_setup(context, *args, **kwargs):
        # Evaluate the 'config_file' argument
        config_file_path = LaunchConfiguration('config_file').perform(context)

        # Load the default parameters
        with open(_DEFAULT_PARAMS_FILE, 'r') as default_file:
            default_params = yaml.safe_load(default_file)

        # Load the custom parameters
        with open(config_file_path, 'r') as custom_file:
            custom_params = yaml.safe_load(custom_file)

        # Merge parameters (custom params override default params)
        merged_params = {**default_params, **custom_params}

        # Define the Microstrain sensor node
        microstrain_node = Node(
            package=_PACKAGE_NAME,
            executable="microstrain_inertial_driver_node",
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            parameters=[merged_params]
        )

        return [microstrain_node]

    # Add OpaqueFunction to handle dynamic parameter loading
    launch_description.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description)
