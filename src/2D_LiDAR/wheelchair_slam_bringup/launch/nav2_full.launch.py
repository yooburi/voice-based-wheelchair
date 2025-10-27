#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('wheelchair_slam_bringup')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params_full.yaml')

    # Use Nav2's default BTs with replanning + recoveries
    nav2_bt_share = get_package_share_directory('nav2_bt_navigator')
    default_bt_pose = os.path.join(
        nav2_bt_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )
    default_bt_through_poses = os.path.join(
        nav2_bt_share, 'behavior_trees', 'navigate_w_replanning_only_if_path_becomes_invalid.xml'
    )

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full Nav2 parameters YAML'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map YAML for nav2_map_server'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo/Sim time'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Autostart lifecycle nodes'),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {"yaml_filename": map_yaml, 'use_sim_time': use_sim_time}],
        ),

        # AMCL (localization)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Planner Server (Smac Hybrid-A*)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Controller Server (Regulated Pure Pursuit)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Behavior Server (for recovery behaviors)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # BT Navigator (ComputePathToPose + FollowPath + Recoveries)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    #'default_nav_to_pose_bt_xml': default_bt_pose,
                    'default_nav_through_poses_bt_xml': default_bt_through_poses,
                },
            ],
        ),

        # Lifecycle Manager to autostart nodes in sequence
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': [
                        'map_server',
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'bt_navigator',
                    ],
                }
            ],
        ),

        # Optional: RViz preset (uncomment if desired)
        # Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(pkg_share, '../../..', 'dolchair.rviz')]),
    ])
