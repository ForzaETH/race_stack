"""
A Python launch file is required here to cleanly shutdown the planner after it has
computed and saved the global waypoints.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rs_launch.launch_extensions import CheckArgs


def generate_launch_description():
    map_name = LaunchConfiguration('map')
    map_path = PathJoinSubstitution([EnvironmentVariable('MAPS_DIR'), map_name])

    # Compute and store the global plan of the map, then exit
    planner = Node(
        package='rs_global_planner',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('stack_master'), 'config', 'global_planner',
                                  'global_planner_params.yaml']),
            {'map_path': map_path},
        ])

    # Serve the stored plan, the same way the base system does
    plan_server = Node(
        package='rs_global_planner',
        executable='global_plan_server',
        name='global_plan_server',
        output='screen',
        parameters=[{'map_path': map_path}])

    def on_planner_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='global planner failed')]
        return [plan_server]

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='latest',
                              description='Map at $MAPS_DIR/map to compute the global plan for'),

        # fail on arguments this launch file does not declare
        CheckArgs(),

        planner,
        RegisterEventHandler(OnProcessExit(target_action=planner, on_exit=on_planner_exit)),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': PathJoinSubstitution([map_path, [map_name, '.yaml']])}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]),

        # Sector slicer: shutdown everything once sectors are defined
        Node(
            package='rs_sectors',
            executable='sector_slicer',
            name='sector_slicer',
            output='screen',
            on_exit=Shutdown(),
            parameters=[{'map_path': map_path}]),
    ])
