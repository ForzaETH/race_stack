import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    # Launch argument to set car_name namespace
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'sensor_interface',
            default_value='ethernet',
            description='sensor_interface: supported: serial, ethernet'
        ),
    ])

    urg_node_dir = get_package_share_directory('urg_node')
    car_name = os.getenv('F1TENTH_CAR_NAME', '')

    # Function to dynamically expand parameter file based on sensor_interface
    def expand_param_file_name(context):
        param_file = os.path.join(
            urg_node_dir, 'launch',
            'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml')
        if os.path.exists(param_file):
            return [SetLaunchConfiguration('param', param_file)]

    param_file_path = OpaqueFunction(function=expand_param_file_name)
    launch_description.add_action(param_file_path)

    # Namespace from launch argument
    car_name_namespace = LaunchConfiguration('car_name')
    base_link_frame = 'base_link'
    laser_frame = 'laser'
    if len(car_name) != 0:
        base_link_frame = base_link_frame + '_' + car_name
        laser_frame = laser_frame + '_' + car_name
    # urg_node driver with namespace
    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        output='screen',
        namespace=car_name,  # Apply namespace
        parameters=[LaunchConfiguration('param'),
                    {
                    'laser_frame_id': laser_frame,  # Append car_name to frame_id
                    }
                    ]
    )
    launch_description.add_action(hokuyo_node)

    # Static transform from base_link to laser
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser2',
        namespace=car_name,  # Apply namespace
        arguments=[
            '0', '0', '0',  # Translation: x, y, z
            '0', '0', '0',  # Rotation (roll, pitch, yaw in radians, 180 degrees roll)
            base_link_frame, laser_frame  # Frame IDs: parent, child
        ]
    )
    launch_description.add_action(static_tf_node)

    return launch_description
