"""
Traffic Light Obedience — No Depth Gating (Ablation) — ROSbot 3 PRO (Project B)

Launches the system with depth-based spatial gating DISABLED.
Used for the ablation study: comparing detection accuracy with vs. without depth gating.

Usage:
  ros2 launch traffic_light_nav traffic_light_nav_no_depth.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('traffic_light_nav')
    config_file = os.path.join(pkg_dir, 'config', 'traffic_params.yaml')

    signal_detector_node = Node(
        package='traffic_light_nav',
        executable='signal_detector',
        name='signal_detector',
        parameters=[
            config_file,
            {'use_depth_gating': False},  # DISABLED for ablation
        ],
        output='screen',
    )

    navigation_controller_node = Node(
        package='traffic_light_nav',
        executable='navigation_controller',
        name='navigation_controller',
        parameters=[config_file],
        output='screen',
    )

    data_logger_node = Node(
        package='traffic_light_nav',
        executable='traffic_logger',
        name='traffic_logger',
        parameters=[
            config_file,
            {'log_directory': '/tmp/traffic_light_logs/no_depth_gating'},
        ],
        output='screen',
    )

    return LaunchDescription([
        signal_detector_node,
        navigation_controller_node,
        data_logger_node,
    ])
