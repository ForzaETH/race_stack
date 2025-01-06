from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare the racecar_version argument
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value='NUC1',  # NUC1, ORINNANO1
        description='Version of the racecar'
    )

    # Access the racecar_version
    racecar_version = LaunchConfiguration('racecar_version')

    # Base path for the stack_master package config
    config_base_path = PathJoinSubstitution([
        FindPackageShare('stack_master'),
        'config',
        racecar_version,
    ])

    # YAML file paths
    urg_node_yaml = PathJoinSubstitution([config_base_path, 'urg_node.yaml'])
    imu_yaml = PathJoinSubstitution([config_base_path, 'imu.yaml'])

    # Find package directories
    urg_node_pkg = FindPackageShare('urg_node').find('urg_node')
    vesc_pkg = FindPackageShare('vesc').find('vesc')
    imu_pkg = FindPackageShare('microstrain_inertial_examples').find('microstrain_inertial_examples')

    # Launch file paths
    urg_node_launch = os.path.join(urg_node_pkg, 'launch', 'urg_node_f1tenth_launch.py')
    vesc_launch = os.path.join(vesc_pkg, 'launch', 'vesc_f110.launch.py')
    imu_launch = os.path.join(imu_pkg, 'launch', 'gx5_25_launch.py')

    return LaunchDescription([
        # Declare the racecar_version argument
        racecar_version_arg,

        # Include other launch files with YAML arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_node_launch),
            launch_arguments={'config_file': urg_node_yaml}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vesc_launch),
            launch_arguments={'config_folder': config_base_path}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            launch_arguments={'config_file': imu_yaml}.items()
        ),
    ])
