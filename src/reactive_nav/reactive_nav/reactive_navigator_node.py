"""
Reactive Navigator Node — Reactive Autonomous Navigation (ROSbot 3 PRO)

Implements a purely reactive navigation policy that combines:
  1. Virtual Force Field (VFF) — repulsive forces from obstacles + forward
     driving force → resultant vector determines heading and speed.
  2. Behavioural State Machine — overrides VFF output for specific scenarios
     that require structured responses (dead-end recovery, narrow passages).

States
------
  FREE_ROAM         — No nearby obstacles; cruise at full speed with random
                      exploration bias to maximise area coverage.
  CAREFUL           — Obstacles within influence range; VFF actively steering.
  NARROW_PASSAGE    — Left and right sectors blocked but front is clear;
                      reduce speed and straighten heading.
  AVOIDING          — Front sector critically close; hard turn away.
  DEAD_END_RECOVERY — Front + left + right blocked; back up then rotate.
  EMERGENCY_STOP    — External override from safety monitor (not managed here).

Subscribed topics:
  /obstacle_sectors  — Float32MultiArray[12]: min distance per sector
  /dynamic_obstacles — Float32MultiArray[12]: 1.0 if dynamic, 0.0 if static

Published topics:
  /nav_cmd_vel       — geometry_msgs/Twist (sent to safety monitor gate)
  /nav_state         — std_msgs/String (current state for logging/debugging)
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


class NavigatorState:
    FREE_ROAM = 'FREE_ROAM'
    CAREFUL = 'CAREFUL'
    NARROW_PASSAGE = 'NARROW_PASSAGE'
    AVOIDING = 'AVOIDING'
    DEAD_END_RECOVERY = 'DEAD_END_RECOVERY'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


class ReactiveNavigatorNode(Node):
    def __init__(self):
        super().__init__('reactive_navigator')

        # --- Parameters ---
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('narrow_passage_speed', 0.12)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('backup_speed', -0.15)
        self.declare_parameter('free_roam_threshold', 1.2)
        self.declare_parameter('careful_threshold', 0.8)
        self.declare_parameter('narrow_threshold', 0.5)
        self.declare_parameter('avoid_threshold', 0.35)
        self.declare_parameter('dead_end_threshold', 0.5)
        self.declare_parameter('critical_threshold', 0.2)
        self.declare_parameter('drive_gain', 1.0)
        self.declare_parameter('repulsive_gain', 0.8)
        self.declare_parameter('influence_distance', 1.5)
        self.declare_parameter('damping', 0.3)
        self.declare_parameter('random_bias_interval', 4.0)
        self.declare_parameter('random_bias_magnitude', 0.3)
        self.declare_parameter('backup_duration', 2.0)
        self.declare_parameter('rotate_duration', 2.5)
        self.declare_parameter('rotate_speed', 1.0)
        self.declare_parameter('dynamic_obstacle_margin', 0.6)
        self.declare_parameter('dynamic_avoidance_gain', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)

        # Cache parameters
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.min_lin = self.get_parameter('min_linear_speed').value
        self.narrow_speed = self.get_parameter('narrow_passage_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.free_thresh = self.get_parameter('free_roam_threshold').value
        self.careful_thresh = self.get_parameter('careful_threshold').value
        self.narrow_thresh = self.get_parameter('narrow_threshold').value
        self.avoid_thresh = self.get_parameter('avoid_threshold').value
        self.dead_end_thresh = self.get_parameter('dead_end_threshold').value
        self.critical_thresh = self.get_parameter('critical_threshold').value
        self.drive_gain = self.get_parameter('drive_gain').value
        self.repulsive_gain = self.get_parameter('repulsive_gain').value
        self.influence_dist = self.get_parameter('influence_distance').value
        self.damping = self.get_parameter('damping').value
        self.bias_interval = self.get_parameter('random_bias_interval').value
        self.bias_mag = self.get_parameter('random_bias_magnitude').value
        self.backup_dur = self.get_parameter('backup_duration').value
        self.rotate_dur = self.get_parameter('rotate_duration').value
        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.dyn_margin = self.get_parameter('dynamic_obstacle_margin').value
        self.dyn_gain = self.get_parameter('dynamic_avoidance_gain').value
        rate_hz = self.get_parameter('control_rate_hz').value

        # --- State ---
        self.state = NavigatorState.FREE_ROAM
        self.sectors = [5.0] * 12
        self.dynamic_flags = [0.0] * 12
        self.num_sectors = 12
        self.sector_width = 2.0 * math.pi / self.num_sectors

        # Smoothed output
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # Random exploration bias
        self.random_bias = 0.0
        self.last_bias_time = time.time()

        # Dead-end recovery timing
        self.recovery_start = None
        self.recovery_phase = None  # 'backup' or 'rotate'
        self.rotate_direction = 1.0

        # --- Subscribers ---
        self.create_subscription(
            Float32MultiArray, '/obstacle_sectors', self._sectors_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/dynamic_obstacles', self._dynamic_cb, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/nav_cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/nav_state', 10)

        # --- Timer ---
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info('Reactive navigator initialised')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _sectors_cb(self, msg: Float32MultiArray):
        if len(msg.data) == self.num_sectors:
            self.sectors = list(msg.data)

    def _dynamic_cb(self, msg: Float32MultiArray):
        if len(msg.data) == self.num_sectors:
            self.dynamic_flags = list(msg.data)

    # ------------------------------------------------------------------
    # Sector helpers
    # ------------------------------------------------------------------

    def _sector_angle(self, idx: int) -> float:
        """Centre angle of sector idx in radians (0=front, CCW positive)."""
        return idx * self.sector_width

    def _front_min(self) -> float:
        """Minimum distance in the front sectors (11, 0, 1)."""
        return min(self.sectors[11], self.sectors[0], self.sectors[1])

    def _left_min(self) -> float:
        """Minimum distance in the left sectors (2, 3)."""
        return min(self.sectors[2], self.sectors[3])

    def _right_min(self) -> float:
        """Minimum distance in the right sectors (9, 10)."""
        return min(self.sectors[9], self.sectors[10])

    def _any_dynamic_front(self) -> bool:
        """Check if any front-facing sector has a dynamic obstacle."""
        return any(self.dynamic_flags[i] > 0.5 for i in [10, 11, 0, 1, 2])

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _update_state(self):
        """Determine current behavioural state from sector distances."""
        front = self._front_min()
        left = self._left_min()
        right = self._right_min()

        # Stay in DEAD_END_RECOVERY until the manoeuvre completes
        if self.state == NavigatorState.DEAD_END_RECOVERY:
            if self.recovery_phase is not None:
                return  # let the recovery run

        # Dead-end: front + left + right all blocked
        if (front < self.dead_end_thresh and
                left < self.dead_end_thresh and
                right < self.dead_end_thresh):
            if self.state != NavigatorState.DEAD_END_RECOVERY:
                self.state = NavigatorState.DEAD_END_RECOVERY
                self.recovery_start = time.time()
                self.recovery_phase = 'backup'
                # Rotate toward the side with more space
                self.rotate_direction = 1.0 if left > right else -1.0
                self.get_logger().info('DEAD_END detected — starting recovery')
            return

        # Avoiding: front critically close
        if front < self.avoid_thresh:
            self.state = NavigatorState.AVOIDING
            return

        # Narrow passage: sides close but front relatively clear
        if (left < self.narrow_thresh and right < self.narrow_thresh and
                front > self.careful_thresh):
            self.state = NavigatorState.NARROW_PASSAGE
            return

        # Careful: at least one sector within influence range
        min_all = min(self.sectors)
        if min_all < self.careful_thresh:
            self.state = NavigatorState.CAREFUL
            return

        # Free roam
        self.state = NavigatorState.FREE_ROAM

    # ------------------------------------------------------------------
    # VFF computation
    # ------------------------------------------------------------------

    def _compute_vff(self) -> tuple:
        """Compute Virtual Force Field resultant → (linear_vel, angular_vel)."""
        # Forward driving force (in robot frame: +x is forward)
        fx = self.drive_gain
        fy = 0.0

        for i in range(self.num_sectors):
            dist = self.sectors[i]
            if dist < self.influence_dist and dist > 0.01:
                angle = self._sector_angle(i)
                # Inverse-distance repulsive magnitude
                magnitude = self.repulsive_gain * (1.0 / dist - 1.0 / self.influence_dist)

                # Boost repulsion for dynamic obstacles
                if self.dynamic_flags[i] > 0.5:
                    magnitude *= self.dyn_gain

                # Force points AWAY from the obstacle
                fx += magnitude * (-math.cos(angle))
                fy += magnitude * (-math.sin(angle))

        # Resultant direction and magnitude
        result_angle = math.atan2(fy, fx)
        result_mag = math.sqrt(fx * fx + fy * fy)

        # Map to cmd_vel
        linear = min(self.max_lin, max(self.min_lin, result_mag * self.max_lin / (self.drive_gain + 1.0)))
        angular = max(-self.max_ang, min(self.max_ang, result_angle * 2.0))

        # Reduce speed when turning hard
        turn_factor = 1.0 - 0.6 * abs(angular) / self.max_ang
        linear *= max(0.2, turn_factor)

        return linear, angular

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        """Main control loop — runs at control_rate_hz."""
        self._update_state()

        linear = 0.0
        angular = 0.0

        if self.state == NavigatorState.FREE_ROAM:
            linear, angular = self._free_roam()
        elif self.state == NavigatorState.CAREFUL:
            linear, angular = self._careful()
        elif self.state == NavigatorState.NARROW_PASSAGE:
            linear, angular = self._narrow_passage()
        elif self.state == NavigatorState.AVOIDING:
            linear, angular = self._avoiding()
        elif self.state == NavigatorState.DEAD_END_RECOVERY:
            linear, angular = self._dead_end_recovery()

        # Smooth output with exponential filter
        alpha = 1.0 - self.damping
        linear = alpha * linear + self.damping * self.prev_linear
        angular = alpha * angular + self.damping * self.prev_angular
        self.prev_linear = linear
        self.prev_angular = angular

        # Publish
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    # ------------------------------------------------------------------
    # Behaviours
    # ------------------------------------------------------------------

    def _free_roam(self) -> tuple:
        """Cruise forward with random exploration bias for area coverage."""
        # Update random bias periodically
        now = time.time()
        if now - self.last_bias_time > self.bias_interval:
            self.random_bias = random.uniform(-self.bias_mag, self.bias_mag)
            self.last_bias_time = now

        return self.max_lin, self.random_bias

    def _careful(self) -> tuple:
        """Navigate using VFF — obstacles are in influence range."""
        return self._compute_vff()

    def _narrow_passage(self) -> tuple:
        """Slow down, centre between the two sides."""
        left = self._left_min()
        right = self._right_min()

        # Steer toward the side with more space to centre the robot
        balance = (left - right) / max(left + right, 0.01)
        angular = balance * 0.5  # gentle centering correction

        return self.narrow_speed, angular

    def _avoiding(self) -> tuple:
        """Front is critically close — hard turn away from nearest front obstacle."""
        # Determine which front side is more blocked
        left_front = min(self.sectors[1], self.sectors[2])
        right_front = min(self.sectors[10], self.sectors[11])

        # Turn away from the closer side
        if left_front < right_front:
            angular = -self.max_ang  # turn right
        else:
            angular = self.max_ang   # turn left

        # Very slow forward (or zero if extremely close)
        front = self._front_min()
        if front < self.critical_thresh:
            linear = 0.0
        else:
            linear = self.min_lin

        return linear, angular

    def _dead_end_recovery(self) -> tuple:
        """Back up, then rotate to find open space."""
        elapsed = time.time() - self.recovery_start

        if self.recovery_phase == 'backup':
            if elapsed < self.backup_dur:
                return self.backup_speed, 0.0
            else:
                self.recovery_phase = 'rotate'
                self.recovery_start = time.time()
                self.get_logger().info('Dead-end recovery: switching to ROTATE')
                return 0.0, 0.0

        if self.recovery_phase == 'rotate':
            elapsed = time.time() - self.recovery_start
            if elapsed < self.rotate_dur:
                return 0.0, self.rotate_speed * self.rotate_direction
            else:
                # Recovery complete — reset
                self.recovery_phase = None
                self.recovery_start = None
                self.state = NavigatorState.CAREFUL
                self.get_logger().info('Dead-end recovery COMPLETE')
                return 0.0, 0.0

        return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigatorNode()
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
