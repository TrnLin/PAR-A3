"""
Sensor Fusion Node — Reactive Autonomous Navigation (ROSbot 3 PRO)

Fuses three sensor modalities into a unified 12-sector obstacle representation:
  1. S2 LIDAR (360° LaserScan) — primary, fills all sectors
  2. OAK-D Pro depth camera (~72° FOV) — secondary, fills front sectors
  3. VL53L0X ToF x4 (point range) — supplements specific sectors

Each sector spans 30° and stores the minimum obstacle distance detected by
any sensor. Sector 0 is centred at 0° (dead ahead), increasing counter-clockwise.

Also performs temporal differencing on per-sector distances to detect dynamic
obstacles (rapid decrease in distance indicates an approaching object).

Published topics:
  /obstacle_sectors    — Float32MultiArray[12]: min distance per sector
  /dynamic_obstacles   — Float32MultiArray[12]: 1.0 if dynamic, 0.0 if static
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, Range
from std_msgs.msg import Float32MultiArray


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # --- Parameters ---
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('tof_front_left_topic', '/range/fl')
        self.declare_parameter('tof_front_right_topic', '/range/fr')
        self.declare_parameter('tof_rear_left_topic', '/range/rl')
        self.declare_parameter('tof_rear_right_topic', '/range/rr')
        self.declare_parameter('num_sectors', 12)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_valid_range', 0.05)
        self.declare_parameter('depth_camera_hfov', 72.0)
        self.declare_parameter('depth_max_range', 4.0)
        self.declare_parameter('depth_min_range', 0.2)
        self.declare_parameter('use_depth_camera', True)
        self.declare_parameter('dynamic_detect_threshold', 0.3)
        self.declare_parameter('sector_history_window', 5)
        self.declare_parameter('fusion_rate_hz', 20.0)

        self.num_sectors = self.get_parameter('num_sectors').value
        self.max_range = self.get_parameter('max_range').value
        self.min_valid_range = self.get_parameter('min_valid_range').value
        self.depth_hfov = math.radians(self.get_parameter('depth_camera_hfov').value)
        self.depth_max = self.get_parameter('depth_max_range').value
        self.depth_min = self.get_parameter('depth_min_range').value
        self.use_depth = self.get_parameter('use_depth_camera').value
        self.dynamic_threshold = self.get_parameter('dynamic_detect_threshold').value
        self.history_window = self.get_parameter('sector_history_window').value
        rate_hz = self.get_parameter('fusion_rate_hz').value

        self.sector_width = 2.0 * math.pi / self.num_sectors  # radians per sector

        # --- Sensor data buffers ---
        self.lidar_sectors = np.full(self.num_sectors, self.max_range)
        self.depth_sectors = np.full(self.num_sectors, self.max_range)
        self.tof_sectors = np.full(self.num_sectors, self.max_range)

        # History for dynamic obstacle detection (deque per sector)
        self.sector_history = [
            deque(maxlen=self.history_window) for _ in range(self.num_sectors)
        ]

        # --- ToF sensor angular positions (approximate mounting on ROSbot 3 PRO) ---
        # Front-left ~30°, Front-right ~-30°, Rear-left ~150°, Rear-right ~-150°
        self.tof_angles = {
            'fl': math.radians(20),
            'fr': math.radians(-20),
            'rl': math.radians(160),
            'rr': math.radians(-160),
        }
        self.tof_latest = {'fl': self.max_range, 'fr': self.max_range,
                           'rl': self.max_range, 'rr': self.max_range}

        # --- Subscribers ---
        self.create_subscription(
            LaserScan,
            self.get_parameter('lidar_topic').value,
            self._lidar_cb,
            SENSOR_QOS,
        )

        if self.use_depth:
            self.create_subscription(
                Image,
                self.get_parameter('depth_image_topic').value,
                self._depth_cb,
                SENSOR_QOS,
            )
            self.get_logger().info('Depth camera ENABLED for sensor fusion')
        else:
            self.get_logger().info('Depth camera DISABLED (LIDAR-only ablation mode)')

        # ToF subscriptions
        for key, param_name in [('fl', 'tof_front_left_topic'),
                                 ('fr', 'tof_front_right_topic'),
                                 ('rl', 'tof_rear_left_topic'),
                                 ('rr', 'tof_rear_right_topic')]:
            topic = self.get_parameter(param_name).value
            self.create_subscription(
                Range, topic,
                lambda msg, k=key: self._tof_cb(msg, k),
                SENSOR_QOS,
            )

        # --- Publishers ---
        self.sector_pub = self.create_publisher(Float32MultiArray, '/obstacle_sectors', 10)
        self.dynamic_pub = self.create_publisher(Float32MultiArray, '/dynamic_obstacles', 10)

        # --- Timer ---
        self.create_timer(1.0 / rate_hz, self._fuse_and_publish)

        self.get_logger().info(
            f'Sensor fusion initialised: {self.num_sectors} sectors, '
            f'depth={"ON" if self.use_depth else "OFF"}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _lidar_cb(self, msg: LaserScan):
        """Process 360° LIDAR scan into sector minimum distances."""
        sectors = np.full(self.num_sectors, self.max_range)
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_valid_range < r < self.max_range:
                sector_idx = self._angle_to_sector(angle)
                sectors[sector_idx] = min(sectors[sector_idx], r)
            angle += msg.angle_increment
        self.lidar_sectors = sectors

    def _depth_cb(self, msg: Image):
        """Process depth image into sector minimum distances for the camera FOV."""
        sectors = np.full(self.num_sectors, self.max_range)

        # Depth image is 16UC1 (millimetres) or 32FC1 (metres)
        if msg.encoding == '16UC1':
            dtype = np.uint16
            scale = 0.001  # mm → m
        elif msg.encoding == '32FC1':
            dtype = np.float32
            scale = 1.0
        else:
            return  # unsupported encoding

        depth_array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        depth_m = depth_array.astype(np.float32) * scale

        # Sample a horizontal strip at the middle of the image (where obstacles
        # at robot height are most likely to appear)
        strip_top = msg.height // 3
        strip_bot = 2 * msg.height // 3
        strip = depth_m[strip_top:strip_bot, :]

        # For each column, compute the minimum valid depth
        col_min = np.full(msg.width, self.max_range)
        for col in range(msg.width):
            col_data = strip[:, col]
            valid = col_data[(col_data > self.depth_min) & (col_data < self.depth_max)]
            if len(valid) > 0:
                col_min[col] = np.min(valid)

        # Map columns to angles and then to sectors
        # Column 0 = left edge of FOV, column width-1 = right edge
        for col in range(msg.width):
            angle = (col / msg.width - 0.5) * self.depth_hfov  # negative=right, positive=left
            if self.depth_min < col_min[col] < self.depth_max:
                sector_idx = self._angle_to_sector(angle)
                sectors[sector_idx] = min(sectors[sector_idx], col_min[col])

        self.depth_sectors = sectors

    def _tof_cb(self, msg: Range, key: str):
        """Store the latest ToF range reading."""
        if msg.range > msg.min_range:
            self.tof_latest[key] = min(msg.range, self.max_range)
        else:
            self.tof_latest[key] = msg.min_range

    # ------------------------------------------------------------------
    # Fusion
    # ------------------------------------------------------------------

    def _fuse_and_publish(self):
        """Merge all sensor sectors via per-sector minimum, detect dynamics, publish."""
        # Build ToF sector array
        tof_sectors = np.full(self.num_sectors, self.max_range)
        for key, angle in self.tof_angles.items():
            idx = self._angle_to_sector(angle)
            tof_sectors[idx] = min(tof_sectors[idx], self.tof_latest[key])

        # Fused = per-sector minimum across all three modalities
        fused = np.minimum(self.lidar_sectors, tof_sectors)
        if self.use_depth:
            fused = np.minimum(fused, self.depth_sectors)

        # --- Dynamic obstacle detection ---
        dynamic_flags = np.zeros(self.num_sectors)
        for i in range(self.num_sectors):
            self.sector_history[i].append(fused[i])
            if len(self.sector_history[i]) >= 3:
                history = list(self.sector_history[i])
                # Compute rate of distance decrease (m per tick)
                rates = [history[j] - history[j + 1] for j in range(len(history) - 1)]
                avg_rate = sum(rates) / len(rates)
                if avg_rate > self.dynamic_threshold:
                    dynamic_flags[i] = 1.0

        # Publish fused sectors
        sector_msg = Float32MultiArray()
        sector_msg.data = fused.tolist()
        self.sector_pub.publish(sector_msg)

        # Publish dynamic flags
        dynamic_msg = Float32MultiArray()
        dynamic_msg.data = dynamic_flags.tolist()
        self.dynamic_pub.publish(dynamic_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _angle_to_sector(self, angle: float) -> int:
        """Convert an angle (radians, 0=front, CCW positive) to a sector index."""
        # Normalise to [0, 2π)
        angle = angle % (2.0 * math.pi)
        # Offset by half-sector so sector 0 is centred on 0°
        angle = (angle + self.sector_width / 2.0) % (2.0 * math.pi)
        idx = int(angle / self.sector_width)
        return min(idx, self.num_sectors - 1)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
