import os
import shutil
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import re

def ensure_permissions_and_path(file_path):
    """Ensure the file exists, has correct permissions, and can be accessed."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    # Set file permissions to 644
    os.chmod(file_path, 0o644)

    # Ensure the directory has the correct permissions
    dir_path = os.path.dirname(file_path)
    os.chmod(dir_path, 0o755)


def generate_launch_description():
    # 기본 경로 설정
    cartographer_shared_dir = get_package_share_directory('cartographer_easystart')
    cartographer_config_dir = os.path.join(cartographer_shared_dir, 'config')


    multi_agent_dir = os.path.join(cartographer_config_dir, 'multi_agent')

    map_filename = 'rita1.pbstream'
    lua_file = 'localization_2d.lua'
    pbstream_file = os.path.join(cartographer_shared_dir, 'maps', map_filename)

    car_name = os.getenv('F1TENTH_CAR_NAME', '')  # 기본값 설정

    if len(car_name) != 0:
        original_lua_file = os.path.join(cartographer_config_dir, lua_file)
        lua_file = lua_file[:-4] + f'_{car_name}.lua'
        cartographer_config_dir = os.path.join(cartographer_config_dir, 'multi_agent')
        modified_lua_file = os.path.join(multi_agent_dir, lua_file)
        os.makedirs(multi_agent_dir, exist_ok=True)

        with open(original_lua_file, 'r') as src, open(modified_lua_file, 'w') as dst:
            for line in src:
                # 필요한 키 수정
                # for key in ["map_frame", "tracking_frame", "published_frame", "odom_frame"]:
                for key in [ "tracking_frame", "published_frame", "odom_frame"]:
                    if f"{key} =" in line:  # 키 이름 뒤에 " = " 형식이 있는지 확인
                        # " " 안의 문자열 추출 및 수정
                        start_idx = line.find('"') + 1  # 시작 인덱스 (첫 번째 " 다음)
                        end_idx = line.find('"', start_idx)  # 종료 인덱스 (다음 ")
                        if start_idx != 0 and end_idx != -1:  # 유효한 인덱스인지 확인
                            original_value = line[start_idx:end_idx]  # 원래 값 추출
                            modified_value = f"{original_value}_{car_name}"  # 수정된 값 생성
                            # " " 내부 문자열만 수정
                            line = f"{line[:start_idx]}{modified_value}{line[end_idx:]}"
                        break  # 한 줄에서 여러 키 중복 처리를 방지
                dst.write(line)

        # 파일 및 경로 권한 보장
        ensure_permissions_and_path(modified_lua_file)

    # Cartographer 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        namespace=car_name,
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', lua_file,
            '-load_state_filename', pbstream_file,  # Load the saved map
        ],
        remappings=[
            ('imu', 'gx5/imu/data')  # IMU 데이터 리매핑
        ],
    )

    # Occupancy Grid 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        namespace=car_name,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ],
    )

    # Carto Init Pose 노드
    carto_init_pose_node = Node(
        package='cartographer_easystart',
        executable='initialize.py',
        namespace=car_name,
        name='initialize_py_node',
        output='screen',
        parameters=[
            {'configuration_directory': cartographer_config_dir},  # 설정 디렉토리 전달
            {'configuration_basename': lua_file}  # Lua 파일명 전달
        ],
        remappings=[
            ('/initialpose', '/initialpose'),
            ('/move_base_simple/goal', '/move_base_simple/goal')
        ],
    )

    carto_state_estimator = Node(
        package='cartographer_easystart',
        executable='state_estimator',
        namespace=car_name,
        name='carto_state_estimator',
        output='screen',
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        carto_init_pose_node,
        carto_state_estimator,
    ])
