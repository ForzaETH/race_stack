from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    custom_ekf = LaunchConfiguration('custom_ekf')
    ekf_node = None
    if custom_ekf == 'true':
        # Covariance matrices. Kept as plain literals (not launch args) since ROS2
        # launch arguments are always strings and can't cleanly carry float arrays.
        # Q needs to match the state-dimension of the selected model.
        ekf_node = Node(
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
                    ),
    else:
        ekf_node = Node(
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
        ekf_node
    ])
