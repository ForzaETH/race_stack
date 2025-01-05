import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Cartographer 설정 디렉토리와 파일 경로 설정
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_easystart'),
        'config')
    lua_file = 'mapping_2d.lua'  # 올바른 lua 파일명으로 수정

    # Cartographer 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', lua_file
        ],
        remappings=[
            ('/imu', '/gx5/imu/data')  # 기존 '/imu'를 '/new_imu_topic'으로 remap
        ],
    )

    # Occupancy Grid 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',  # Occupancy Grid의 해상도 설정
            '-publish_period_sec', '1.0'
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])
