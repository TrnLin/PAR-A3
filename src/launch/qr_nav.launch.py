"""
QR Code Command Navigation Launch — ROSbot 3 PRO (Project A)

Launches QR detector, command interpreter, and data logger.

Usage:
  ros2 launch qr_nav qr_nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('qr_nav')
    config_file = os.path.join(pkg_dir, 'config', 'qr_params.yaml')

    qr_detector_node = Node(
        package='qr_nav',
        executable='qr_detector',
        name='qr_detector',
        parameters=[config_file],
        output='screen',
    )

    command_interpreter_node = Node(
        package='qr_nav',
        executable='command_interpreter',
        name='command_interpreter',
        parameters=[config_file],
        output='screen',
    )

    data_logger_node = Node(
        package='qr_nav',
        executable='qr_logger',
        name='qr_logger',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        qr_detector_node,
        command_interpreter_node,
        data_logger_node,
    ])
