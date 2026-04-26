"""
Full System Launch — Reactive Autonomous Navigation (ROSbot 3 PRO)

Launches all four nodes with full sensor fusion (LIDAR + depth + ToF).
For the LIDAR-only ablation study, use reactive_nav_lidar_only.launch.py
or pass use_depth_camera:=false.

Usage:
  ros2 launch reactive_nav reactive_nav_full.launch.py

  # Override depth camera (ablation):
  ros2 launch reactive_nav reactive_nav_full.launch.py use_depth_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('reactive_nav')
    config_file = os.path.join(pkg_dir, 'config', 'nav_params.yaml')

    use_depth_arg = DeclareLaunchArgument(
        'use_depth_camera',
        default_value='true',
        description='Enable OAK-D Pro depth camera. Set false for LIDAR-only ablation.',
    )

    sensor_fusion_node = Node(
        package='reactive_nav',
        executable='sensor_fusion',
        name='sensor_fusion',
        parameters=[
            config_file,
            {'use_depth_camera': LaunchConfiguration('use_depth_camera')},
        ],
        output='screen',
    )

    reactive_navigator_node = Node(
        package='reactive_nav',
        executable='reactive_navigator',
        name='reactive_navigator',
        parameters=[config_file],
        output='screen',
    )

    safety_monitor_node = Node(
        package='reactive_nav',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[config_file],
        output='screen',
    )

    data_logger_node = Node(
        package='reactive_nav',
        executable='data_logger',
        name='data_logger',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        use_depth_arg,
        sensor_fusion_node,
        reactive_navigator_node,
        safety_monitor_node,
        data_logger_node,
    ])
