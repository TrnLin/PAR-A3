#!/usr/bin/env python3
"""
Navigation controller combining LIDAR wall-following with traffic-light signal response.

Implements a signal-state machine (GREEN / YELLOW / RED / UNKNOWN) and uses
LIDAR sectors for corridor navigation and obstacle avoidance.
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NavigationControllerNode(Node):
    """Drives the ROSbot based on signal state and LIDAR readings."""

    def __init__(self):
        super().__init__('navigation_controller_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('cruise_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('front_clear_distance', 0.8)
        self.declare_parameter('emergency_stop_distance', 0.2)
        self.declare_parameter('wall_follow_gain', 1.0)
        self.declare_parameter('unknown_timeout', 3.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('lidar_topic', '/scan')

        self._cruise = self.get_parameter('cruise_speed').value
        self._max_ang = self.get_parameter('max_angular_speed').value
        self._front_clear = self.get_parameter('front_clear_distance').value
        self._e_stop = self.get_parameter('emergency_stop_distance').value
        self._wall_gain = self.get_parameter('wall_follow_gain').value
        self._unknown_timeout = self.get_parameter('unknown_timeout').value
        self._rate_hz = self.get_parameter('control_rate_hz').value
        lidar_topic = self.get_parameter('lidar_topic').value

        # ── Internal state ──────────────────────────────────────────────
        self._signal_state = 'UNKNOWN'
        self._last_known_state = 'RED'  # safe default before first signal
        self._signal_stamp = self.get_clock().now()
        self._unknown_since = None  # timestamp when UNKNOWN started
        self._latest_scan = None
        self._nav_state = 'STOPPED'

        # Latency tracking
        self._signal_received_time = None  # wall-clock time of last signal msg
        self._last_cmd_signal_state = None  # signal state at last cmd_vel publish

        # ── QoS ─────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            String, '/signal_state', self._signal_callback, 10)
        self.create_subscription(
            LaserScan, lidar_topic, self._scan_callback, sensor_qos)

        # ── Publishers ──────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_pub = self.create_publisher(String, '/nav_state', 10)

        # ── Control loop timer ──────────────────────────────────────────
        self.create_timer(1.0 / self._rate_hz, self._control_loop)

        self.get_logger().info(
            f'NavigationControllerNode started – cruise: {self._cruise} m/s, '
            f'control rate: {self._rate_hz} Hz')

    # ── Callbacks ───────────────────────────────────────────────────────

    def _signal_callback(self, msg: String):
        new_state = msg.data.upper()
        if new_state not in ('RED', 'YELLOW', 'GREEN', 'UNKNOWN'):
            self.get_logger().warn(f'Ignoring invalid signal state: {msg.data}')
            return

        now = time.time()
        self._signal_received_time = now

        if new_state != self._signal_state:
            self.get_logger().info(
                f'Signal state received: {self._signal_state} -> {new_state}')

        if new_state != 'UNKNOWN':
            self._last_known_state = new_state
            self._unknown_since = None
        else:
            if self._signal_state != 'UNKNOWN':
                self._unknown_since = now

        self._signal_state = new_state
        self._signal_stamp = self.get_clock().now()

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    # ── LIDAR sector helpers ────────────────────────────────────────────

    @staticmethod
    def _sector_min(ranges, angle_min, angle_inc, sector_start_deg, sector_end_deg):
        """Return the minimum valid range in a given angular sector (degrees)."""
        vals = []
        for i, r in enumerate(ranges):
            angle_deg = math.degrees(angle_min + i * angle_inc)
            if sector_start_deg <= angle_deg <= sector_end_deg:
                if math.isfinite(r) and r > 0.01:
                    vals.append(r)
        return min(vals) if vals else float('inf')

    @staticmethod
    def _sector_mean(ranges, angle_min, angle_inc, sector_start_deg, sector_end_deg):
        """Return the mean valid range in a given angular sector."""
        vals = []
        for i, r in enumerate(ranges):
            angle_deg = math.degrees(angle_min + i * angle_inc)
            if sector_start_deg <= angle_deg <= sector_end_deg:
                if math.isfinite(r) and r > 0.01:
                    vals.append(r)
        return (sum(vals) / len(vals)) if vals else float('inf')

    # ── Main control loop ───────────────────────────────────────────────

    def _control_loop(self):
        twist = Twist()

        # ── Determine effective signal behaviour ────────────────────────
        effective_state = self._signal_state

        if effective_state == 'UNKNOWN':
            if self._unknown_since is not None:
                elapsed = time.time() - self._unknown_since
                if elapsed > self._unknown_timeout:
                    # Default to YELLOW (slow) after timeout
                    effective_state = 'YELLOW'
                else:
                    effective_state = self._last_known_state
            else:
                effective_state = self._last_known_state

        # ── Signal-based speed factor ───────────────────────────────────
        if effective_state == 'RED':
            target_linear = 0.0
            nav_label = 'STOPPED_RED'
        elif effective_state == 'YELLOW':
            target_linear = self._cruise * 0.5
            nav_label = 'SLOW_YELLOW'
        else:  # GREEN
            target_linear = self._cruise
            nav_label = 'DRIVING_GREEN'

        # ── LIDAR navigation (only when moving) ─────────────────────────
        angular_z = 0.0

        if target_linear > 0.0 and self._latest_scan is not None:
            scan = self._latest_scan
            a_min = scan.angle_min
            a_inc = scan.angle_increment
            ranges = scan.ranges

            # Sectors: left-front (30..90°), front (-30..30°), right-front (-90..-30°)
            front_min = self._sector_min(ranges, a_min, a_inc, -30.0, 30.0)
            left_mean = self._sector_mean(ranges, a_min, a_inc, 30.0, 90.0)
            right_mean = self._sector_mean(ranges, a_min, a_inc, -90.0, -30.0)

            # Emergency stop
            overall_front_min = self._sector_min(ranges, a_min, a_inc, -45.0, 45.0)
            if overall_front_min < self._e_stop:
                target_linear = 0.0
                angular_z = 0.0
                nav_label = 'EMERGENCY_STOP'
            elif front_min < self._front_clear:
                # Front blocked – turn toward the side with more space
                if left_mean > right_mean:
                    angular_z = self._max_ang  # turn left (positive)
                else:
                    angular_z = -self._max_ang  # turn right
                target_linear *= 0.3  # slow down while turning
                nav_label += '_TURNING'
            else:
                # Corridor centering: proportional steering
                if left_mean < float('inf') and right_mean < float('inf'):
                    error = left_mean - right_mean
                    angular_z = self._wall_gain * error
                    angular_z = max(-self._max_ang, min(self._max_ang, angular_z))

        # Clamp angular velocity
        angular_z = max(-self._max_ang, min(self._max_ang, angular_z))

        twist.linear.x = target_linear
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

        # ── Response latency logging ────────────────────────────────────
        if (self._signal_received_time is not None and
                self._signal_state != self._last_cmd_signal_state):
            latency_ms = (time.time() - self._signal_received_time) * 1000.0
            self.get_logger().info(
                f'Response latency for {self._signal_state}: {latency_ms:.1f} ms')
            self._last_cmd_signal_state = self._signal_state

        # ── Publish nav state ───────────────────────────────────────────
        if nav_label != self._nav_state:
            self.get_logger().info(f'Nav state: {self._nav_state} -> {nav_label}')
            self._nav_state = nav_label

        nav_msg = String()
        nav_msg.data = self._nav_state
        self._nav_pub.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocity on shutdown
        stop = Twist()
        node._cmd_pub.publish(stop)
        node.get_logger().info('Shutting down – sent zero velocity.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
