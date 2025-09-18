#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    with_rf2o = LaunchConfiguration('with_rf2o').perform(context).lower() == 'true'
    with_ekf = LaunchConfiguration('with_ekf').perform(context).lower() == 'true'

    pkg_share = get_package_share_directory('wheelchair_slam_bringup')
    default_urdf = os.path.join(pkg_share, 'urdf', 'rplidar_myahrs.urdf')
    default_slam = os.path.join(pkg_share, 'config', 'slam.yaml')
    default_ekf = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')

    urdf_file = LaunchConfiguration('robot_description_file').perform(context) or default_urdf
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context) or default_slam
    ekf_params_file = LaunchConfiguration('ekf_params_file').perform(context) or default_ekf

    # Resolve to absolute paths and read URDF
    if not os.path.isabs(urdf_file):
        urdf_file = os.path.join(pkg_share, urdf_file)
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    if not os.path.isabs(slam_params_file):
        slam_params_file = os.path.join(pkg_share, slam_params_file)
    if not os.path.isabs(ekf_params_file):
        ekf_params_file = os.path.join(pkg_share, ekf_params_file)

    nodes = []

    # Static TFs from URDF
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # LiDAR odometry (rf2o) to provide /odom
    if with_rf2o:
        nodes.append(Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ))

    # EKF fusion (optional)
    if with_ekf:
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
        ))

    # SLAM Toolbox (async)
    nodes.append(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_slam_bringup')
    default_urdf = os.path.join(pkg_share, 'urdf', 'rplidar_myahrs.urdf')
    default_slam = os.path.join(pkg_share, 'config', 'slam.yaml')
    default_ekf = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated time (rosbag --clock)'),
        DeclareLaunchArgument('with_rf2o', default_value='true', description='Run rf2o laser odometry'),
        DeclareLaunchArgument('with_ekf', default_value='false', description='Run robot_localization EKF fusion'),
        DeclareLaunchArgument('robot_description_file', default_value=default_urdf, description='URDF path'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam, description='slam_toolbox params YAML'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf, description='robot_localization EKF params YAML'),
        OpaqueFunction(function=_launch_setup),
    ])

