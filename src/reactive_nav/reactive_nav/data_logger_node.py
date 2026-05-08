"""
Data Logger Node — Reactive Autonomous Navigation (ROSbot 3 PRO)

Logs all relevant data for post-run evaluation:
  - Timestamps
  - Robot position and velocity (from odometry)
  - Obstacle sector distances (fused)
  - Dynamic obstacle flags
  - Navigator state
  - Safety status
  - Collision events (sector distance below contact threshold)

Output: CSV files in the configured log directory, one per run.

Evaluation metrics computed offline from the logs:
  - Collision rate (contacts per minute)
  - Coverage (area traversed from odometry path)
  - State distribution (time in each state)
  - Dynamic obstacle response latency
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')

        # --- Parameters ---
        self.declare_parameter('log_directory', '/tmp/reactive_nav_logs')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('collision_distance_threshold', 0.08)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        log_dir = self.get_parameter('log_directory').value
        rate_hz = self.get_parameter('log_rate_hz').value
        self.collision_thresh = self.get_parameter('collision_distance_threshold').value

        # --- Create log directory and file ---
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(log_dir, f'nav_log_{timestamp}.csv')

        self.csv_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        sector_headers = [f'sector_{i}' for i in range(12)]
        dynamic_headers = [f'dynamic_{i}' for i in range(12)]
        self.csv_writer.writerow([
            'timestamp', 'elapsed_s',
            'pos_x', 'pos_y', 'yaw',
            'linear_vel', 'angular_vel',
            *sector_headers, *dynamic_headers,
            'nav_state', 'safety_status',
            'collision_event', 'collision_sector',
            'cmd_linear_x', 'cmd_angular_z',
        ])

        # --- State buffers ---
        self.start_time = time.time()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.sectors = [5.0] * 12
        self.dynamic_flags = [0.0] * 12
        self.nav_state = 'UNKNOWN'
        self.safety_status = 'UNKNOWN'
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # Collision tracking
        self.total_collisions = 0
        self.last_collision_time = 0.0

        # Coverage tracking
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        # --- Subscribers ---
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_cb, 10,
        )
        self.create_subscription(
            Float32MultiArray, '/obstacle_sectors', self._sectors_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/dynamic_obstacles', self._dynamic_cb, 10)
        self.create_subscription(
            String, '/nav_state', self._state_cb, 10)
        self.create_subscription(
            String, '/safety_status', self._safety_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 10)

        # --- Timer ---
        self.create_timer(1.0 / rate_hz, self._log_tick)

        self.get_logger().info(f'Data logger writing to: {self.log_path}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

        # Update coverage distance
        if self.prev_x is not None:
            dx = self.pos_x - self.prev_x
            dy = self.pos_y - self.prev_y
            self.total_distance += math.sqrt(dx * dx + dy * dy)
        self.prev_x = self.pos_x
        self.prev_y = self.pos_y

    def _sectors_cb(self, msg: Float32MultiArray):
        if len(msg.data) == 12:
            self.sectors = list(msg.data)

    def _dynamic_cb(self, msg: Float32MultiArray):
        if len(msg.data) == 12:
            self.dynamic_flags = list(msg.data)

    def _state_cb(self, msg: String):
        self.nav_state = msg.data

    def _safety_cb(self, msg: String):
        self.safety_status = msg.data

    def _cmd_cb(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_tick(self):
        now = time.time()
        elapsed = now - self.start_time

        # Detect collision events (debounced to 1 per second)
        collision_event = False
        collision_sector = -1
        if now - self.last_collision_time > 1.0:
            for i, dist in enumerate(self.sectors):
                if dist < self.collision_thresh:
                    collision_event = True
                    collision_sector = i
                    self.total_collisions += 1
                    self.last_collision_time = now
                    self.get_logger().warn(
                        f'COLLISION detected in sector {i} (dist={dist:.3f}m) '
                        f'— total: {self.total_collisions}'
                    )
                    break

        # Write CSV row
        self.csv_writer.writerow([
            f'{now:.3f}', f'{elapsed:.2f}',
            f'{self.pos_x:.4f}', f'{self.pos_y:.4f}', f'{self.yaw:.4f}',
            f'{self.linear_vel:.4f}', f'{self.angular_vel:.4f}',
            *[f'{s:.3f}' for s in self.sectors],
            *[f'{d:.0f}' for d in self.dynamic_flags],
            self.nav_state, self.safety_status,
            int(collision_event), collision_sector,
            f'{self.cmd_linear:.4f}', f'{self.cmd_angular:.4f}',
        ])
        self.csv_file.flush()

        # Periodic summary
        if int(elapsed) % 30 == 0 and int(elapsed) > 0:
            minutes = elapsed / 60.0
            rate = self.total_collisions / minutes if minutes > 0 else 0
            self.get_logger().info(
                f'[{elapsed:.0f}s] Collisions: {self.total_collisions} '
                f'({rate:.2f}/min), Distance: {self.total_distance:.1f}m, '
                f'State: {self.nav_state}'
            )

    def destroy_node(self):
        # Final summary
        elapsed = time.time() - self.start_time
        minutes = elapsed / 60.0
        rate = self.total_collisions / minutes if minutes > 0 else 0
        self.get_logger().info(
            f'=== FINAL SUMMARY ===\n'
            f'  Duration: {elapsed:.1f}s ({minutes:.1f} min)\n'
            f'  Total collisions: {self.total_collisions}\n'
            f'  Collision rate: {rate:.2f}/min\n'
            f'  Distance covered: {self.total_distance:.1f}m\n'
            f'  Log file: {self.log_path}'
        )
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
