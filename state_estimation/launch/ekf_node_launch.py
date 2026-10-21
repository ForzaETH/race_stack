import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Covariance sets per model. Kept as plain literals (not launch args) since ROS2
# launch arguments are always strings and can't cleanly carry float arrays.
# Q needs to match the state dimension of the selected model.
DEFAULT_COV = {
    'R_imu': [0.1, 0.1, 1000000.0, 1000000.0],
    'R_vesc': [1, 1, 1, 1, 100, 0.1],
    'R_vio': [0.1, 0.1, 0.1, 0.1, 0.1, 1000000.0],
    'Q': [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
}

# single_track_full_model: state
# [x, y, theta, vx, vy, omega, s_w, b_gz, b_ax, b_ay, d_x, d_y, d_psi]
# Starting values only: shrink Q on d_* as the tire fit improves. Keep Q on
# vx, vy, omega small: process noise should enter through d_*, not the velocities.
WHEEL_MODEL_COV = {
    'R_imu': [0.1, 0.001, 1.0, 1.0],                    # yaw, gyro, ax, ay (accel no longer disabled)
    'R_vesc': [1, 1, 1, 1, 100, 0.1],                   # unused: VESC speed is an input
    'R_vio': [0.1, 0.1, 0.1, 0.01, 0.01, 1000000.0],    # x, y, yaw, vx, vy, yaw rate
    'Q': [0.001, 0.001, 0.001, 1e-4, 1e-4, 1e-3,
          1e-7, 1e-8, 1e-6, 1e-6,
          0.5, 0.5, 0.01],
}

WHEEL_MODEL_PARAMS = {
    'gate_prob': 0.99,
    'gate_max_reject': 50,
    'imu_acc_ema_alpha': 1.0,        # raw accel to the measurement; ax_lt carries the low-passed signal
    'tau_lt': 0.03,
    'steer_lag': True,
    'imu_px': 0.0,                   # TODO: VESC/IMU position from CoG [m]
    'imu_py': 0.0,
    'base_link_at_rear_axle': True,
    'n_substeps': 4,
    'C_roll': 0.0,                   # TODO: from coast-down
    'C_drag': 0.0,
}


def launch_setup(context, *args, **kwargs):
    model_type = LaunchConfiguration('model_type').perform(context)
    is_wheel = model_type == 'single_track_full_model'
    cov = WHEEL_MODEL_COV if is_wheel else DEFAULT_COV

    params = {
        'frequency': 100,
        'racecar_version': LaunchConfiguration('racecar_version'),
        'model_type': model_type,
        'odom_topic': LaunchConfiguration('odom_topic'),
        'imu_topic': LaunchConfiguration('imu_topic'),
        'vesc_topic': LaunchConfiguration('odom_vesc_topic'),
        'vio_topic': '/basalt/odom',
        'floor': LaunchConfiguration('floor'),
        **cov,
    }
    if is_wheel:
        params.update(WHEEL_MODEL_PARAMS)

    custom_ekf_node = Node(
                        condition=IfCondition(LaunchConfiguration('custom_ekf')),
                        package='state_estimation',
                        executable='ekf_node',
                        name='ekf_node',
                        output='screen',
                        emulate_tty=True,
                        parameters=[params],
                    )

    robot_localization_ekf_node = Node(
                        condition=UnlessCondition(LaunchConfiguration('custom_ekf')),
                        package='robot_localization',
                        executable='ekf_node',
                        name='ekf_filter_node',
                        output='screen',
                        parameters=[os.path.join(get_package_share_directory("state_estimation"), 'config', 'ekf.yaml')],
                        remappings=[('/odometry/filtered', '/state_estimation/odom'),]
                    )
    return [custom_ekf_node, robot_localization_ekf_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('custom_ekf', default_value='false'),
        DeclareLaunchArgument('model_type', default_value='point_mass_model'),
        DeclareLaunchArgument('racecar_version', default_value='none'),
        DeclareLaunchArgument('floor', default_value='dubi'),

        DeclareLaunchArgument('odom_vesc_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation/odom'),
        OpaqueFunction(function=launch_setup),
    ])
