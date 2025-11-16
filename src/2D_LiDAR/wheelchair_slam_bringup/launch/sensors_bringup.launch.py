#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def include_pkg_launch(pkg: str, launch_file: str, launch_arguments=None) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        launch_arguments=(launch_arguments.items() if isinstance(launch_arguments, dict) else ())
    )


def generate_launch_description() -> LaunchDescription:
    # Optional arguments
    imu_cfg = LaunchConfiguration('imu_config')
    imu_use_rviz = LaunchConfiguration('imu_use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('imu_config', default_value='imu_bias.yaml', description='IMU preprocess config file'),
        DeclareLaunchArgument('imu_use_rviz', default_value='false', description='Open IMU RViz during driver launch'),

        # 1) 2D LiDAR (SLLIDAR)
        include_pkg_launch('sllidar_ros2', 'view_sllidar_a2m8_launch.py'),

        # 2) RealSense D435i (custom wrapper launch under this repo)
        include_pkg_launch('wheelchair_slam_bringup', 'rs_camera.launch.py'),

        # 3) MYAHRS+ (IMU driver)
        include_pkg_launch('myahrs_ros2_driver', 'myahrs_ros2_driver.launch.py', {
            'use_rviz': imu_use_rviz,
        }),

        # 4) IMU preprocess (bias, filter, republish)
        include_pkg_launch('imu_preprocess', 'imu_preprocess.launch.py', {
            'config': imu_cfg,
        }),
    ])

