#!/usr/bin/env python3
"""
Data logger node for the Traffic Light Navigation (Project B).

Records signal state, raw detections, navigation state, velocity commands,
odometry, and response latency to a timestamped CSV file.
"""

import csv
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class DataLoggerNode(Node):
    """Logs experiment data for post-run analysis."""

    def __init__(self):
        super().__init__('data_logger_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('log_directory', '/tmp/traffic_light_logs')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        self._log_dir = self.get_parameter('log_directory').value
        self._rate_hz = self.get_parameter('log_rate_hz').value
        odom_topic = self.get_parameter('odom_topic').value

        # ── Create log directory and file ───────────────────────────────
        os.makedirs(self._log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(self._log_dir, f'traffic_log_{timestamp_str}.csv')

        self._csv_file = open(self._log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp',
            'elapsed_s',
            'signal_state',
            'raw_detections',
            'nav_state',
            'cmd_linear_x',
            'cmd_angular_z',
            'pos_x',
            'pos_y',
            'response_latency_ms',
        ])

        # ── Internal state ──────────────────────────────────────────────
        self._start_time = time.time()

        self._signal_state = 'UNKNOWN'
        self._raw_detections = ''
        self._nav_state = ''
        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0

        # Response latency tracking
        self._last_signal_change_time = None
        self._last_signal_for_latency = 'UNKNOWN'
        self._latest_response_latency_ms = 0.0
        self._cmd_vel_received_after_change = False

        # Per-color detection counts
        self._color_counts = {'RED': 0, 'YELLOW': 0, 'GREEN': 0, 'UNKNOWN': 0}
        self._state_duration = {'RED': 0.0, 'YELLOW': 0.0, 'GREEN': 0.0, 'UNKNOWN': 0.0}
        self._last_state_time = time.time()

        # Latency statistics
        self._latency_samples = []

        # ── QoS for sensor topics ───────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            String, '/signal_state', self._signal_state_cb, 10)
        self.create_subscription(
            String, '/signal_detections', self._signal_detections_cb, 10)
        self.create_subscription(
            String, '/nav_state', self._nav_state_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            Odometry, odom_topic, self._odom_cb, sensor_qos)

        # ── Logging timer ───────────────────────────────────────────────
        self.create_timer(1.0 / self._rate_hz, self._log_tick)

        self.get_logger().info(
            f'DataLoggerNode started – logging to {self._log_path} at {self._rate_hz} Hz')

    # ── Subscriber callbacks ────────────────────────────────────────────

    def _signal_state_cb(self, msg: String):
        new_state = msg.data
        now = time.time()

        # Track state duration
        if self._signal_state != new_state:
            dt = now - self._last_state_time
            self._state_duration[self._signal_state] = (
                self._state_duration.get(self._signal_state, 0.0) + dt)
            self._last_state_time = now

            # Detection count
            if new_state in self._color_counts:
                self._color_counts[new_state] += 1

            # Latency: record time of signal change
            self._last_signal_change_time = now
            self._cmd_vel_received_after_change = False

        self._signal_state = new_state

    def _signal_detections_cb(self, msg: String):
        self._raw_detections = msg.data

    def _nav_state_cb(self, msg: String):
        self._nav_state = msg.data

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_linear_x = msg.linear.x
        self._cmd_angular_z = msg.angular.z

        # Compute response latency on first cmd_vel after signal change
        if (self._last_signal_change_time is not None and
                not self._cmd_vel_received_after_change):
            latency = (time.time() - self._last_signal_change_time) * 1000.0
            self._latest_response_latency_ms = round(latency, 2)
            self._latency_samples.append(self._latest_response_latency_ms)
            self._cmd_vel_received_after_change = True

    def _odom_cb(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

    # ── Periodic CSV logging ────────────────────────────────────────────

    def _log_tick(self):
        now = time.time()
        elapsed = round(now - self._start_time, 3)

        self._csv_writer.writerow([
            round(now, 6),
            elapsed,
            self._signal_state,
            self._raw_detections,
            self._nav_state,
            round(self._cmd_linear_x, 4),
            round(self._cmd_angular_z, 4),
            round(self._pos_x, 4),
            round(self._pos_y, 4),
            self._latest_response_latency_ms,
        ])
        self._csv_file.flush()

    # ── Shutdown summary ────────────────────────────────────────────────

    def destroy_node(self):
        # Flush final state duration
        now = time.time()
        dt = now - self._last_state_time
        self._state_duration[self._signal_state] = (
            self._state_duration.get(self._signal_state, 0.0) + dt)

        total_time = now - self._start_time

        self.get_logger().info('=' * 60)
        self.get_logger().info('DATA LOGGER SUMMARY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Log file: {self._log_path}')
        self.get_logger().info(f'Total run time: {total_time:.1f} s')
        self.get_logger().info('--- Detection counts ---')
        for color in ['RED', 'YELLOW', 'GREEN', 'UNKNOWN']:
            self.get_logger().info(f'  {color}: {self._color_counts[color]} transitions')
        self.get_logger().info('--- Signal state distribution ---')
        for state, dur in self._state_duration.items():
            pct = (dur / total_time * 100.0) if total_time > 0 else 0.0
            self.get_logger().info(f'  {state}: {dur:.1f} s ({pct:.1f}%)')
        if self._latency_samples:
            avg_lat = sum(self._latency_samples) / len(self._latency_samples)
            max_lat = max(self._latency_samples)
            min_lat = min(self._latency_samples)
            self.get_logger().info(f'--- Response latency (ms) ---')
            self.get_logger().info(
                f'  avg: {avg_lat:.1f}, min: {min_lat:.1f}, max: {max_lat:.1f}, '
                f'samples: {len(self._latency_samples)}')
        else:
            self.get_logger().info('No response latency samples recorded.')
        self.get_logger().info('=' * 60)

        self._csv_file.close()
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
