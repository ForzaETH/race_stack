# -----------------------------------------------------------------------------
# Basalt VIO -> EKF bridge. The RealSense D435 driver itself is launched with
# the other sensors in stack_master/launch/teleop_launch.xml (USE_VIO=true).
# Basalt calibration/VIO config lives in the state_estimation package.
# Component layout based on Bernd Pfrommer <bernd.pfrommer@gmail.com>
# -----------------------------------------------------------------------------
import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
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
    """Dynamically set up parameters and assemble VIO nodes."""

    # --- Basalt VIO container ---
    racecar_version = LaunchConfig('racecar_version').perform(context)
    calib_file_name = f"d435_calib_{racecar_version}.json"
    config_file_name = "vio_config.json"

    common_parameters = {
        'calibration_file': state_estimation_file(calib_file_name),
        'vio_config_file': state_estimation_file(config_file_name)
    }

    frontend_parameters = common_parameters
    backend_parameters = {
        **common_parameters,
        'world_frame_id': 'odom',                  # REP-105: VIO outputs to drifting odom frame
        'publish_tf': False,                       # Let EKF handle global transform publishing
        'has_split_accel_and_gyro_topics': False    # D435 outputs a unified combined IMU topic
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

    # --- Optional feature-track visualization (/basalt/flow_image) ---
    viz_flow_node = Node(
        package='basalt_ros',
        executable='viz_flow_node',
        name='viz_flow',
        namespace='basalt',
        output='screen',
        parameters=[{'show_tracks': True}],
        remappings=[
            ('left_image', LaunchConfig('left_image_topic')),
            ('optical_flow', 'optical_flow')               # frontend output, /basalt/optical_flow
        ],
        condition=IfCondition(LaunchConfig('viz_flow')),
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
            'imu_topic': LaunchConfig('imu_topic'),
            'tf_timeout': LaunchConfig('tf_timeout')
        }]
    )

    return [vio_container, basalt_to_ekf_node, viz_flow_node]


def generate_launch_description():
    """Declare all args, then assemble the VIO stack via OpaqueFunction
    (needed because racecar_version has to be resolved at runtime to build
    the calibration filename)."""

    return launch.LaunchDescription([
        LaunchArg('camera_name',
                  default_value='camera',
                  description='Name/namespace the RealSense node is launched under (see teleop_launch.xml)'),

        # --- Basalt / EKF bridge args ---
        LaunchArg('racecar_version',
                  default_value='NUC6',
                  description='Selects config/d435_calib_<racecar_version>.json'),
        LaunchArg('left_image_topic',
                  default_value=camera_topic('infra1', 'image_rect_raw'),
                  description='Left infra camera raw image topic'),
        LaunchArg('right_image_topic',
                  default_value=camera_topic('infra2', 'image_rect_raw'),
                  description='Right infra camera raw image topic'),
        LaunchArg('imu_topic',
                  default_value=camera_topic('imu'),
                  description='Combined RealSense IMU data topic'),
        LaunchArg('odom_topic',
                  default_value='/basalt/odom',
                  description='EKF-ready target destination odometry topic'),
        LaunchArg('tf_timeout',
                  default_value='30.0',
                  description='Seconds the bridge node yields for driver transforms to wake up'),
        LaunchArg('viz_flow',
                  default_value='false',
                  description='Publish /basalt/flow_image with tracked features drawn on the left image'),

        OpaqueFunction(function=launch_setup),
    ])
