"""
Traffic Light Obedience Full Launch — ROSbot 3 PRO (Project B)

Launches signal detector (with depth gating), navigation controller, and logger.

Usage:
  ros2 launch traffic_light_nav traffic_light_nav_full.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('traffic_light_nav')
    config_file = os.path.join(pkg_dir, 'config', 'traffic_params.yaml')

    use_depth_arg = DeclareLaunchArgument(
        'use_depth_gating',
        default_value='true',
        description='Enable depth-based spatial gating. Set false for ablation.',
    )

    signal_detector_node = Node(
        package='traffic_light_nav',
        executable='signal_detector',
        name='signal_detector',
        parameters=[
            config_file,
            {'use_depth_gating': LaunchConfiguration('use_depth_gating')},
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
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        use_depth_arg,
        signal_detector_node,
        navigation_controller_node,
        data_logger_node,
    ])
