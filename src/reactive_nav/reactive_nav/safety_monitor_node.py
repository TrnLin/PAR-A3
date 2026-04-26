"""
Safety Monitor Node — Reactive Autonomous Navigation (ROSbot 3 PRO)

Acts as a safety gate between the reactive navigator and the motor controller.
Subscribes to the four VL53L0X ToF sensors and to /nav_cmd_vel (from the
navigator). If ANY ToF sensor reads below the emergency-stop threshold, a
zero-velocity command is published to /cmd_vel regardless of the navigator's
output. Once all ToF readings recover above the resume threshold, normal
forwarding resumes.

This node runs at a higher rate than the navigator (50 Hz by default) because
it is safety-critical.

Published topics:
  /cmd_vel         — geometry_msgs/Twist (final motor commands)
  /safety_status   — std_msgs/String ('OK' or 'EMERGENCY_STOP')
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # --- Parameters ---
        self.declare_parameter('emergency_stop_distance', 0.10)
        self.declare_parameter('resume_distance', 0.25)
        self.declare_parameter('tof_timeout', 0.5)
        self.declare_parameter('tof_front_left_topic', '/range/fl')
        self.declare_parameter('tof_front_right_topic', '/range/fr')
        self.declare_parameter('nav_cmd_vel_topic', '/nav_cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('monitor_rate_hz', 50.0)

        self.stop_dist = self.get_parameter('emergency_stop_distance').value
        self.resume_dist = self.get_parameter('resume_distance').value
        self.tof_timeout = self.get_parameter('tof_timeout').value
        rate_hz = self.get_parameter('monitor_rate_hz').value

        # --- State ---
        self.is_stopped = False
        self.latest_nav_cmd = Twist()  # last received navigator command

        # ToF readings: {key: (distance, timestamp)}
        self.tof_readings = {
            'fl': (float('inf'), 0.0),
            'fr': (float('inf'), 0.0),
        }

        # --- Subscribers ---
        for key, param_name in [('fl', 'tof_front_left_topic'),
                                 ('fr', 'tof_front_right_topic')]:
            topic = self.get_parameter(param_name).value
            self.create_subscription(
                Range, topic,
                lambda msg, k=key: self._tof_cb(msg, k),
                SENSOR_QOS,
            )

        self.create_subscription(
            Twist,
            self.get_parameter('nav_cmd_vel_topic').value,
            self._nav_cmd_cb,
            10,
        )

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.status_pub = self.create_publisher(String, '/safety_status', 10)

        # --- Timer ---
        self.create_timer(1.0 / rate_hz, self._monitor_loop)

        self.get_logger().info(
            f'Safety monitor initialised: stop < {self.stop_dist:.2f}m, '
            f'resume > {self.resume_dist:.2f}m'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _tof_cb(self, msg: Range, key: str):
        self.tof_readings[key] = (msg.range, time.time())

    def _nav_cmd_cb(self, msg: Twist):
        self.latest_nav_cmd = msg

    # ------------------------------------------------------------------
    # Monitor loop
    # ------------------------------------------------------------------

    def _monitor_loop(self):
        now = time.time()
        any_critical = False
        all_safe = True

        for key, (dist, ts) in self.tof_readings.items():
            age = now - ts
            if age > self.tof_timeout:
                continue  # stale reading — assume safe (no sensor data)

            if dist < self.stop_dist:
                any_critical = True
            if dist < self.resume_dist:
                all_safe = False

        # State transitions with hysteresis
        if any_critical and not self.is_stopped:
            self.is_stopped = True
            self.get_logger().warn(
                f'EMERGENCY STOP — ToF reading below {self.stop_dist:.2f}m'
            )
        elif self.is_stopped and all_safe:
            self.is_stopped = False
            self.get_logger().info('Emergency stop CLEARED — resuming navigation')

        # Publish
        status_msg = String()
        if self.is_stopped:
            # Publish zero velocity
            self.cmd_pub.publish(Twist())
            status_msg.data = 'EMERGENCY_STOP'
        else:
            # Forward navigator command
            self.cmd_pub.publish(self.latest_nav_cmd)
            status_msg.data = 'OK'

        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop on shutdown
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
