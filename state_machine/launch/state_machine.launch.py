import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stack_master'),
        'config',
        'state_machine_params.yaml'
        )
    
    ld = LaunchDescription([
    Node(
        package = 'state_machine',
        name = 'state_machine',
        executable = 'state_machine',
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info'],
    )])
    
    return ld
