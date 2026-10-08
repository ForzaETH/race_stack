import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Both nodes are declared, the condition picks one at launch time (custom_ekf is only resolved then)
    custom_ekf = LaunchConfiguration('custom_ekf')

    # Covariance matrices. Kept as plain literals (not launch args) since ROS2
    # launch arguments are always strings and can't cleanly carry float arrays.
    # Q needs to match the state-dimension of the selected model.
    custom_ekf_node = Node(
                        condition=IfCondition(custom_ekf),
                        package='state_estimation',
                        executable='ekf_node',
                        name='ekf_node',
                        output='screen',
                        emulate_tty=True,
                        parameters=[{
                            'frequency': 100,
                            'racecar_version': LaunchConfiguration('racecar_version'),
                            'model_type': LaunchConfiguration('model_type'),
                            'odom_topic': LaunchConfiguration('odom_topic'),
                            'imu_topic': '/imu',
                            'vesc_topic': LaunchConfiguration('odom_vesc_topic'),
                            'vio_topic': '/basalt/odom',
                            'floor': LaunchConfiguration('floor'),
                            'R_imu': [0.1, 0.1, 1000000.0, 1000000.0],
                            'R_vesc': [1, 1, 1, 1, 100, 0.1],
                            'R_vio': [0.1, 0.1, 0.1, 0.1, 0.1, 1000000.0],
                            'Q': [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
                        }],
                    )

    robot_localization_ekf_node = Node(
                        condition=UnlessCondition(custom_ekf),
                        package='robot_localization',
                        executable='ekf_node',
                        name='ekf_filter_node',
                        output='screen',
                        parameters=[os.path.join(get_package_share_directory("state_estimation"), 'config', 'ekf.yaml')],
                        remappings=[('/odometry/filtered', '/state_estimation/odom'),]
                    )
    
    return LaunchDescription([
        DeclareLaunchArgument('custom_ekf', default_value='false'),
        DeclareLaunchArgument('model_type', default_value='point_mass_model'),
        DeclareLaunchArgument('racecar_version', default_value='none'),
        DeclareLaunchArgument('floor', default_value='dubi'),

        DeclareLaunchArgument('odom_vesc_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation/odom'),
        custom_ekf_node,
        robot_localization_ekf_node,
    ])
