#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Map desired settings to rs_launch.py launch arguments
    launch_args = {
        "align_depth": "true",
        "pointcloud.enable": "true",
        "enable_gyro": "true",
        "enable_accel": "true",
        "enable_sync": "true",
        "unite_imu_method": "linear_interpolation",
        # Correct RealSense argument names
        "depth_module.depth_profile": "640x480x30",
        "rgb_camera.color_profile": "640x480x30",
        # Use single-level prefix: /depth_camera/...
        # Empty namespace and camera_name:=depth_camera
        "camera_namespace": "",
        "camera_name": "depth_camera",
    }

    rs_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments=launch_args.items(),
        )
    ])
