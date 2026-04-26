"""
LIDAR-Only Ablation Launch — Reactive Autonomous Navigation (ROSbot 3 PRO)

Launches the full system with the OAK-D Pro depth camera DISABLED.
Used for the sensor ablation study: comparing LIDAR-only vs. LIDAR + depth.

Usage:
  ros2 launch reactive_nav reactive_nav_lidar_only.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('reactive_nav')
    config_file = os.path.join(pkg_dir, 'config', 'nav_params.yaml')

    sensor_fusion_node = Node(
        package='reactive_nav',
        executable='sensor_fusion',
        name='sensor_fusion',
        parameters=[
            config_file,
            {'use_depth_camera': False},  # DISABLED for ablation
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
        parameters=[
            config_file,
            {'log_directory': '/tmp/reactive_nav_logs/lidar_only'},
        ],
        output='screen',
    )

    return LaunchDescription([
        sensor_fusion_node,
        reactive_navigator_node,
        safety_monitor_node,
        data_logger_node,
    ])
