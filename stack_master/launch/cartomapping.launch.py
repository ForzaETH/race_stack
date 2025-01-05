from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Find package directories
    urg_node_pkg = FindPackageShare('urg_node').find('urg_node')
    vesc_pkg = FindPackageShare('vesc').find('vesc')
    imu_pkg = FindPackageShare('microstrain_inertial_examples').find('microstrain_inertial_examples')
    carto_pkg = FindPackageShare('cartographer_easystart').find('cartographer_easystart')

    # Launch file paths
    urg_node_launch = os.path.join(urg_node_pkg, 'launch', 'urg_node_launch.py')
    vesc_launch = os.path.join(vesc_pkg, 'launch', 'vesc.launch.py')
    imu_launch = os.path.join(imu_pkg, 'launch', 'gx5_25_launch.py')
    carto_mapping_launch = os.path.join(carto_pkg, 'launch', 'mapping_2d_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_node_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vesc_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(carto_mapping_launch),
        ),
    ])
