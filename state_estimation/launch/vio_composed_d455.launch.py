# -----------------------------------------------------------------------------
# Full VIO bringup, single file: RealSense D455 driver -> Basalt VIO -> EKF bridge
# Camera config lives in config/d400_vio.yaml (this package).
# Basalt calibration/VIO config lives in the state_estimation package.
# Component layout based on Bernd Pfrommer <bernd.pfrommer@gmail.com>
# -----------------------------------------------------------------------------
import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def state_estimation_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('state_estimation'), 'config', fname])


def camera_topic(*parts):
    """Build /<camera_name>/<parts...> so image/imu topics always track
    whatever camera_name the RealSense node is actually launched under."""
    subs = [TextSubstitution(text='/'), LaunchConfig('camera_name')]
    subs += [TextSubstitution(text='/'), LaunchConfig('camera_name')]
    for p in parts:
        subs += [TextSubstitution(text='/'), TextSubstitution(text=p)]
    return subs


def launch_setup(context, *args, **kwargs):
    """Dynamically set up parameters and assemble camera + VIO nodes."""

    # --- RealSense camera node ---
    camera_name = LaunchConfig('camera_name')
    serial_no = LaunchConfig('serial_no')
    params_file = LaunchConfig('params_file')

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=camera_name,
        output='screen',
        parameters=[params_file, {'serial_no': serial_no}],
    )

    # --- Basalt VIO container ---
    racecar_version = LaunchConfig('racecar_version').perform(context)
    calib_file_name = f"d455_calib_{racecar_version}.json"
    config_file_name = "d455_vio_config.json"

    common_parameters = {
        'calibration_file': state_estimation_file(calib_file_name),
        'vio_config_file': state_estimation_file(config_file_name)
    }

    frontend_parameters = common_parameters
    backend_parameters = {
        **common_parameters,
        'world_frame_id': 'odom',                  # REP-105: VIO outputs to drifting odom frame
        'child_frame_id': LaunchConfig('imu_frame'),
        'publish_tf': False,                       # Let EKF handle global transform publishing
        'has_split_accel_and_gyro_topics': False    # D455 outputs a unified combined IMU topic
    }

    frontend_remappings = [
        ('left_image', LaunchConfig('left_image_topic')),
        ('right_image', LaunchConfig('right_image_topic'))
    ]

    backend_remappings = [
        ('optical_flow', 'optical_flow'),           # Intra-process link between front/back end
        ('imu', LaunchConfig('imu_topic')),
        ('odom', '/basalt/odomimu')                 # Intermediate topic fed to the EKF bridge
    ]

    vio_container = ComposableNodeContainer(
        name='basalt_vio_container',
        namespace='basalt',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='basalt_ros',
                plugin='basalt_ros::VIOFrontEndNode',
                name='vio_frontend',
                namespace='basalt',
                parameters=[frontend_parameters],
                remappings=frontend_remappings,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='basalt_ros',
                plugin='basalt_ros::VIOBackEndNode',
                name='vio_backend',
                namespace='basalt',
                parameters=[backend_parameters],
                remappings=backend_remappings,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        # Silence the container: 'log' alone would still mirror stderr (where ROS
        # console logging goes) to the screen, so route both streams to the file.
        output={'stdout': 'log', 'stderr': 'log'},
    )

    # --- Basalt -> EKF bridge ---
    basalt_to_ekf_node = Node(
        package='state_estimation',
        executable='basalt_to_ekf_node',
        name='basalt_to_ekf',
        output='screen',
        parameters=[{
            'input_topic': '/basalt/odomimu',
            'output_topic': LaunchConfig('odom_topic'),
            'imu_frame': LaunchConfig('imu_frame'),
            'imu_topic': LaunchConfig('imu_topic'),
            'tf_timeout': LaunchConfig('tf_timeout')
        }]
    )
    
    static_tf_node_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_3dlidar',
        arguments=[
            '--x', '0.262377',
            '--y', '0.043761',
            '--z', '0.120451',
            '--qx', '-0.0003105',
            '--qy', '0.0140676',
            '--qz', '0.0045111',
            '--qw', '0.9998908',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )
    
    static_tf_node_camera_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_link_to_camera_imu',
        arguments=[
            '--x', '-0.016020',
            '--y', '-0.030220',
            '--z', '0.007400',
            '--qx', '-0.5',
            '--qy', '0.5',
            '--qz', '-0.5',
            '--qw', '0.5',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_imu',
        ]
    )

    # return [camera_node, static_tf_node_camera_link, static_tf_node_camera_imu]
    return [camera_node, static_tf_node_camera_link, static_tf_node_camera_imu, vio_container, basalt_to_ekf_node]


def generate_launch_description():
    """Declare all args, then assemble camera + VIO stack via OpaqueFunction
    (needed because racecar_version has to be resolved at runtime to build
    the calibration filename)."""

    default_params_file = state_estimation_file('d455_config.yaml') 

    return launch.LaunchDescription([
        # --- camera identity + config ---
        LaunchArg('camera_name',
                  default_value='camera',
                  description='Name/namespace the RealSense node is launched under'),
        LaunchArg('serial_no',
                  default_value='',
                  description='RealSense device serial number (blank = first device found)'),
        LaunchArg('params_file',
                  default_value=default_params_file,
                  description='YAML file with realsense2_camera_node parameters'),

        # --- Basalt / EKF bridge args ---
        LaunchArg('racecar_version',
                  default_value='NUC6',
                  description='Selects config/d455_calib_<racecar_version>.json'),
        LaunchArg('left_image_topic',
                  default_value=camera_topic('infra1', 'image_rect_raw'),
                  description='Left infra camera raw image topic'),
        LaunchArg('right_image_topic',
                  default_value=camera_topic('infra2', 'image_rect_raw'),
                  description='Right infra camera raw image topic'),
        LaunchArg('imu_topic',
                  default_value=camera_topic('imu'),
                  description='Combined RealSense IMU data topic'),
        LaunchArg('imu_frame',
                  default_value='camera_imu',
                  description='Basalt body frame identity'),
        LaunchArg('odom_topic',
                  default_value='/basalt/odom',
                  description='EKF-ready target destination odometry topic'),
        LaunchArg('tf_timeout',
                  default_value='30.0',
                  description='Seconds the bridge node yields for driver transforms to wake up'),

        OpaqueFunction(function=launch_setup),
    ])