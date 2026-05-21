"""
QR Code Command Navigation Launch — ROSbot 3 PRO (Project A)

Launches QR detector, command interpreter, and data logger.

Usage:
  ros2 launch qr_nav qr_nav.launch.py
  ros2 launch qr_nav qr_nav.launch.py cmd_vel_topic:=/cmd_vel_dummy
  ros2 launch qr_nav qr_nav.launch.py session_id:=t1_per_command
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('qr_nav')
    config_file = os.path.join(pkg_dir, 'config', 'qr_params.yaml')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    session_id = LaunchConfiguration('session_id')

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description=(
            'Topic the command_interpreter publishes to and the logger '
            'listens on. Set to e.g. /cmd_vel_dummy to test without moving '
            'the robot (Stage 2 in DEPLOY.md).'
        ),
    )

    declare_session_id = DeclareLaunchArgument(
        'session_id',
        default_value='',
        description=(
            'Optional trial tag. When non-empty, the data logger writes its '
            'CSV to <repo>/results/<session_id>_<TS>.csv instead of the '
            'auto-grouped session subfolder or the legacy /tmp path. Used '
            'by the experiment workflow in phases/phase-2-experiments.md, '
            'e.g. session_id:=t1_per_command.'
        ),
    )

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
        remappings=[('/cmd_vel', cmd_vel_topic)],
        output='screen',
    )

    data_logger_node = Node(
        package='qr_nav',
        executable='qr_logger',
        name='qr_logger',
        parameters=[config_file, {'session_id': session_id}],
        remappings=[('/cmd_vel', cmd_vel_topic)],
        output='screen',
    )

    return LaunchDescription([
        declare_cmd_vel_topic,
        declare_session_id,
        qr_detector_node,
        command_interpreter_node,
        data_logger_node,
    ])
