import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'nav2_wheel'
    share_dir = get_package_share_directory(pkg_name)

    # URDF/Xacro 파일 경로 설정
    urdf_file_name = 'model.xacro' # <-- 이전에 'model.urdf.xacro' 오류가 났으니, 실제 파일 이름으로 변경
    urdf_path = os.path.join(share_dir, 'urdf', urdf_file_name)

    # 런치 인자 선언
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # === 해결책의 핵심 ===
    # joint_state_publisher를 켤지 말지 결정하는 인자
    use_jsp_arg = DeclareLaunchArgument(
        'use_jsp',
        default_value='true', # 기본값은 true (Rviz 단독 실행시)
        description='Launch joint_state_publisher if true'
    )

    # xacro를 로드하여 robot_description 파라미터 준비
    robot_description_content = Command(['xacro ', urdf_path])

    # 1. robot_state_publisher 노드
    # 이 노드는 항상 필요합니다.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 2. joint_state_publisher 노드 (조건부 실행)
    # === 해결책의 핵심 ===
    # use_jsp 인자가 'true'일 때만 이 노드를 실행합니다.
    # Gazebo 런치 파일에서 이 값을 'false'로 설정할 것입니다.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_jsp'))
    )
   

    return LaunchDescription([
        use_sim_time_arg,
        use_jsp_arg, # 런치 인자 추가
        rsp_node,
        jsp_node,
        rviz_node # 조건부 노드 추가
    ])
