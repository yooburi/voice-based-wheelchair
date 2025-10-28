#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    plan_topic = LaunchConfiguration('plan_topic')
    out_topic = LaunchConfiguration('out_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    points_topic = LaunchConfiguration('points_topic')
    global_frame = LaunchConfiguration('global_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('plan_topic', default_value='/plan'),
        DeclareLaunchArgument('out_topic', default_value='/local_plan'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('points_topic', default_value='/depth_camera/depth/color/points'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),

        # Nav2 path request node (listens to /target_location and publishes /plan)
        Node(
            package='path_planner',
            executable='nav2_pathnplan',
            name='nav2_path_planner',
            output='screen',
            parameters=[
                {'plan_topic': plan_topic},
                {'frame_id': global_frame},
                {'base_frame': base_frame},
            ],
        ),

        # Local avoidance planner (consumes /plan, /scan, /points and publishes /local_plan)
        Node(
            package='path_planner',
            executable='local_avoid_planner',
            name='local_avoid_planner',
            output='screen',
            parameters=[
                {'plan_topic': plan_topic},
                {'out_topic': out_topic},
                {'scan_topic': scan_topic},
                {'points_topic': points_topic},
                {'global_frame': global_frame},
                {'base_frame': base_frame},
            ],
        ),

        # Path follower to track /local_plan
        Node(
            package='path_planner',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[
                {'plan_topic': out_topic},
                {'frame_id': global_frame},
                {'base_frame': base_frame},
            ],
        ),
    ])

