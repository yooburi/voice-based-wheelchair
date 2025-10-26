import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 패키지 공유 디렉터리 경로 찾기
    pkg_dir = get_package_share_directory('motor_driver')

    # 2. 파라미터 YAML 파일 경로 설정
    params_file = os.path.join(pkg_dir, 'config', 'vel_converter_params.yaml')

    # 3. 노드 실행 정의
    vel_converter_node = Node(
        package='motor_driver',
        executable='vel_converter_node', # CMakeLists.txt의 실행 파일 이름
        name='vel_converter_node',       # 노드 이름 (YAML 파일과 일치)
        parameters=[params_file],        # <-- YAML 파일 로드
        output='screen'
    )

    return LaunchDescription([
        vel_converter_node
    ])