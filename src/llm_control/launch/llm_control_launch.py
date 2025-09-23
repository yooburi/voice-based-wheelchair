import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 파라미터 파일의 경로를 설정합니다.
    # 'your_package_name'을 실제 패키지 이름으로 변경해야 합니다.
    # 이 launch 파일과 yaml 파일이 같은 폴더에 있다고 가정합니다.
    pkg_share = get_package_share_directory('llm_control')
    param_file = os.path.join(pkg_share, 'config', 'wheelchair_params.yaml') # 보통 config 폴더에 yaml 파일을 둡니다.

    return LaunchDescription([
        Node(
            package='llm_control',  # 실제 패키지 이름으로 변경
            executable='llm_control', # setup.py에 설정된 실행 파일 이름
            name='llm_control',
            output='screen',
            emulate_tty=True,
            # parameters 키를 사용하여 YAML 파일 로드
            parameters=[param_file]
        )
    ])
