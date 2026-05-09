"""Data Logger Node for QR Code Navigation (Project A).

Subscribes to QR detection, command, navigation state, and velocity topics.
Logs all data to a timestamped CSV file and prints a summary on shutdown.
"""

import csv
import json
import os
import time
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# /cmd_vel on Husarion ROSbot 3 PRO is TwistStamped (Jazzy convention).
# Subscribing with plain Twist would silently never receive callbacks.
from geometry_msgs.msg import TwistStamped


class DataLoggerNode(Node):
    """Logs QR navigation data to CSV for analysis."""

    def __init__(self):
        super().__init__('data_logger_node')

        # Declare parameters
        self.declare_parameter('log_directory', '/tmp/qr_nav_logs')
        self.declare_parameter('log_rate_hz', 5.0)

        # Read parameters
        self.log_directory = (
            self.get_parameter('log_directory').get_parameter_value().string_value
        )
        log_rate_hz = self.get_parameter('log_rate_hz').get_parameter_value().double_value

        # Create log directory if needed
        os.makedirs(self.log_directory, exist_ok=True)

        # Open CSV file
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.log_directory, f'qr_log_{timestamp_str}.csv'
        )
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'elapsed_s',
            'qr_command',
            'all_detections',
            'nav_state',
            'cmd_linear_x',
            'cmd_angular_z',
            'detection_event',
        ])

        # Tracking state
        self.start_time = time.time()
        self.latest_qr_command = ''
        self.latest_all_detections = ''
        self.latest_nav_state = ''
        self.latest_cmd_linear_x = 0.0
        self.latest_cmd_angular_z = 0.0
        self.detection_event = False

        # Statistics
        self.total_commands = 0
        self.command_counts = Counter()
        self.state_time = Counter()  # state -> accumulated seconds
        self.last_state = ''
        self.last_state_time = time.time()

        # Subscribers
        self.qr_command_sub = self.create_subscription(
            String, '/qr_command', self.qr_command_callback, 10
        )
        self.qr_detections_sub = self.create_subscription(
            String, '/qr_detections', self.qr_detections_callback, 10
        )
        self.nav_state_sub = self.create_subscription(
            String, '/nav_state', self.nav_state_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Logging timer
        log_period = 1.0 / log_rate_hz
        self.log_timer = self.create_timer(log_period, self.log_tick)

        self.get_logger().info(
            f'Data Logger Node started — logging to {self.csv_path} '
            f'at {log_rate_hz} Hz'
        )

    def qr_command_callback(self, msg: String):
        """Handle new QR command."""
        self.latest_qr_command = msg.data
        self.detection_event = True
        self.total_commands += 1
        self.command_counts[msg.data] += 1

    def qr_detections_callback(self, msg: String):
        """Handle raw QR detections JSON."""
        self.latest_all_detections = msg.data

    def nav_state_callback(self, msg: String):
        """Handle navigation state updates."""
        now = time.time()
        # Accumulate time in the previous state
        if self.last_state:
            elapsed_in_state = now - self.last_state_time
            self.state_time[self.last_state] += elapsed_in_state

        self.latest_nav_state = msg.data
        self.last_state = msg.data
        self.last_state_time = now

    def cmd_vel_callback(self, msg: TwistStamped):
        """Handle velocity command updates."""
        self.latest_cmd_linear_x = msg.twist.linear.x
        self.latest_cmd_angular_z = msg.twist.angular.z

    def log_tick(self):
        """Write a row to the CSV at the configured rate."""
        now = time.time()
        elapsed = now - self.start_time
        timestamp_str = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]

        self.csv_writer.writerow([
            timestamp_str,
            f'{elapsed:.3f}',
            self.latest_qr_command,
            self.latest_all_detections,
            self.latest_nav_state,
            f'{self.latest_cmd_linear_x:.4f}',
            f'{self.latest_cmd_angular_z:.4f}',
            self.detection_event,
        ])
        self.csv_file.flush()

        # Reset per-tick flags
        self.detection_event = False
        # Clear transient command so it only appears once in the log
        self.latest_qr_command = ''
        self.latest_all_detections = ''

    def print_summary(self):
        """Print a summary of the logged session."""
        # Accumulate time for the final state
        now = time.time()
        if self.last_state:
            elapsed_in_state = now - self.last_state_time
            self.state_time[self.last_state] += elapsed_in_state

        total_elapsed = now - self.start_time

        self.get_logger().info('=' * 60)
        self.get_logger().info('QR Navigation Session Summary')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Log file: {self.csv_path}')
        self.get_logger().info(f'Total duration: {total_elapsed:.1f} s')
        self.get_logger().info(f'Total commands detected: {self.total_commands}')

        if self.command_counts:
            self.get_logger().info('Commands per type:')
            for cmd, count in sorted(self.command_counts.items()):
                self.get_logger().info(f'  {cmd}: {count}')

        if self.state_time:
            self.get_logger().info('State distribution:')
            for state, duration in sorted(self.state_time.items()):
                pct = (duration / total_elapsed * 100) if total_elapsed > 0 else 0.0
                self.get_logger().info(
                    f'  {state}: {duration:.1f}s ({pct:.1f}%)'
                )

        self.get_logger().info('=' * 60)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.print_summary()
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
