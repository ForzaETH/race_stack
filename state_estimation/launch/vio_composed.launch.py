# -----------------------------------------------------------------------------
# RealSense D435 driver + Basalt VIO, run inside the VIO container
# (docker-compose.yaml, service `vio`). Camera and both Basalt stages share one
# component container so images move intra-process, not over the network.
# The Basalt -> EKF bridge runs in the main container (state_estimation_launch.xml).
# Configs are resolved relative to this file, since the VIO container has the
# race_stack sources mounted but not the colcon workspace.
# Component layout based on Bernd Pfrommer <bernd.pfrommer@gmail.com>
# -----------------------------------------------------------------------------
import os

import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

RACE_STACK_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


def state_estimation_file(fname):
    return os.path.join(RACE_STACK_DIR, 'state_estimation', 'config', fname)


def camera_file(fname):
    return os.path.join(RACE_STACK_DIR, 'stack_master', 'config', 'camera', fname)


def camera_topic(*parts):
    """Build /<camera_name>/<camera_name>/<parts...> so image/imu topics always
    track whatever camera_name the RealSense node is launched under."""
    subs = [TextSubstitution(text='/'), LaunchConfig('camera_name')]
    subs += [TextSubstitution(text='/'), LaunchConfig('camera_name')]
    for p in parts:
        subs += [TextSubstitution(text='/'), TextSubstitution(text=p)]
    return subs


def launch_setup(context, *args, **kwargs):
    """Dynamically set up parameters and assemble VIO nodes."""

    racecar_version = LaunchConfig('racecar_version').perform(context)
    camera_model = LaunchConfig('camera_model').perform(context)
    camera_name = LaunchConfig('camera_name').perform(context)

    common_parameters = {
        'calibration_file': state_estimation_file(f"{camera_model}_calib_{racecar_version}.json"),
        'vio_config_file': state_estimation_file('vio_config.json')
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
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=camera_name,
                namespace=camera_name,
                parameters=[camera_file(f"{camera_model}_config.yaml")],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
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
        output='screen',
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

    return [vio_container, viz_flow_node]


def generate_launch_description():
    """Declare all args, then assemble the VIO stack via OpaqueFunction
    (needed because racecar_version has to be resolved at runtime to build
    the calibration filename)."""

    return launch.LaunchDescription([
        LaunchArg('camera_name',
                  default_value='camera',
                  description='Name/namespace of the RealSense node'),
        LaunchArg('camera_model',
                  default_value='d435',
                  description="'d435' / 'd455': selects stack_master/config/camera/<model>_config.yaml "
                              "and state_estimation/config/<model>_calib_<racecar_version>.json"),
        LaunchArg('racecar_version',
                  default_value=os.environ.get('RACECAR_VERSION', 'NUC6'),
                  description='Selects the calibration file'),
        LaunchArg('left_image_topic',
                  default_value=camera_topic('infra1', 'image_rect_raw'),
                  description='Left infra camera raw image topic'),
        LaunchArg('right_image_topic',
                  default_value=camera_topic('infra2', 'image_rect_raw'),
                  description='Right infra camera raw image topic'),
        LaunchArg('imu_topic',
                  default_value=camera_topic('imu'),
                  description='Combined RealSense IMU data topic'),
        LaunchArg('viz_flow',
                  default_value='false',
                  description='Publish /basalt/flow_image with tracked features drawn on the left image'),

        OpaqueFunction(function=launch_setup),
    ])
