import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "nav2_wheel"
    
    # 1. wheelchair.launch.py 포함 (RSP 노드)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "wheelchair_test.launch.py")]
        ),
        # === 해결책의 핵심 ===
        # use_sim_time은 'true'로, use_jsp는 'false'로 설정합니다.
        # 이렇게 하면 Gazebo와 joint_state_publisher가 충돌하지 않습니다.
        launch_arguments={
            'use_sim_time': 'true',
            'use_jsp': 'false'
            
        }.items(),
    )

    # 2. Gazebo 런치 파일 포함
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    )

    # 3. 로봇 스폰 노드
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    
    


    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            
        ]
    )
