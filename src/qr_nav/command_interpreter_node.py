"""Command Interpreter Node with Finite State Machine for QR Code Navigation.

Receives validated QR commands and translates them into velocity commands
for the ROSbot 3 PRO using an FSM with DRIVING, TURNING, STOPPED,
and RECOVERING states.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Husarion ROSbot 3 PRO + ROS 2 Jazzy use TwistStamped on /cmd_vel
# (verified via `ros2 topic info /cmd_vel`). Publishing plain Twist results
# in a silent type mismatch and the base controller ignores us.
from geometry_msgs.msg import TwistStamped


class State:
    """FSM state constants."""
    DRIVING = 'DRIVING'
    TURNING = 'TURNING'
    STOPPED = 'STOPPED'
    RECOVERING = 'RECOVERING'


class CommandInterpreterNode(Node):
    """Interprets QR navigation commands via a finite state machine."""

    def __init__(self):
        super().__init__('command_interpreter_node')

        # Declare parameters
        self.declare_parameter('cruise_speed', 0.2)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('speed_increment', 0.05)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('turn_90_duration', 2.0)
        self.declare_parameter('turn_180_duration', 4.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('recovery_timeout', 5.0)

        # Read parameters
        self.cruise_speed = self.get_parameter('cruise_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.speed_increment = self.get_parameter('speed_increment').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.turn_90_duration = self.get_parameter('turn_90_duration').get_parameter_value().double_value
        self.turn_180_duration = self.get_parameter('turn_180_duration').get_parameter_value().double_value
        control_rate_hz = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        self.recovery_timeout = self.get_parameter('recovery_timeout').get_parameter_value().double_value

        # FSM state
        self.state = State.DRIVING
        self.last_command_time = time.time()

        # Turn tracking
        self.turn_start_time = 0.0
        self.turn_duration = 0.0
        self.turn_angular_z = 0.0

        # Subscriber
        self.command_sub = self.create_subscription(
            String,
            '/qr_command',
            self.command_callback,
            10,
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.nav_state_pub = self.create_publisher(String, '/nav_state', 10)
        # Frame for the stamped twist. Husarion's controller doesn't strictly
        # check this, but rep-105 says base_link is the body frame.
        self.cmd_vel_frame_id = 'base_link'

        # Control loop timer
        control_period = 1.0 / control_rate_hz
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # Publish initial state
        self._publish_state()

        self.get_logger().info(
            f'Command Interpreter Node started — state: {self.state}, '
            f'cruise_speed: {self.cruise_speed} m/s, '
            f'control_rate: {control_rate_hz} Hz'
        )

    def command_callback(self, msg: String):
        """Handle incoming QR commands."""
        command = msg.data.strip()
        now = time.time()
        self.last_command_time = now

        self.get_logger().info(
            f'[{time.strftime("%H:%M:%S")}] Received command: {command} '
            f'(state: {self.state})'
        )

        # TURNING state: ignore all commands until turn completes
        if self.state == State.TURNING:
            self.get_logger().info(
                f'Ignoring command {command} — currently TURNING'
            )
            return

        # STOPPED state: only accept GO
        if self.state == State.STOPPED:
            if command == 'GO':
                self._transition_to(State.DRIVING)
                self.get_logger().info(
                    f'[{time.strftime("%H:%M:%S")}] GO received — '
                    f'resuming at {self.cruise_speed} m/s'
                )
            else:
                self.get_logger().info(
                    f'Ignoring command {command} — currently STOPPED (only GO accepted)'
                )
            return

        # Process commands in DRIVING or RECOVERING state
        self._execute_command(command)

    def _execute_command(self, command: str):
        """Execute a validated QR command."""
        timestamp = time.strftime('%H:%M:%S')

        if command == 'TURN_LEFT':
            self._start_turn(self.turn_speed, self.turn_90_duration)
            self.get_logger().info(
                f'[{timestamp}] Executing TURN_LEFT — '
                f'angular.z={self.turn_speed} for {self.turn_90_duration}s'
            )

        elif command == 'TURN_RIGHT':
            self._start_turn(-self.turn_speed, self.turn_90_duration)
            self.get_logger().info(
                f'[{timestamp}] Executing TURN_RIGHT — '
                f'angular.z={-self.turn_speed} for {self.turn_90_duration}s'
            )

        elif command == 'U_TURN':
            self._start_turn(self.turn_speed, self.turn_180_duration)
            self.get_logger().info(
                f'[{timestamp}] Executing U_TURN — '
                f'angular.z={self.turn_speed} for {self.turn_180_duration}s'
            )

        elif command == 'STOP':
            self._transition_to(State.STOPPED)
            self._publish_velocity(0.0, 0.0)
            self.get_logger().info(f'[{timestamp}] Executing STOP — velocity zeroed')

        elif command == 'GO':
            # GO while already DRIVING/RECOVERING — just ensure DRIVING state
            self._transition_to(State.DRIVING)
            self.get_logger().info(
                f'[{timestamp}] GO received — driving at {self.cruise_speed} m/s'
            )

        elif command == 'SPEED_UP':
            old_speed = self.cruise_speed
            self.cruise_speed = min(
                self.cruise_speed + self.speed_increment, self.max_speed
            )
            self._transition_to(State.DRIVING)
            self.get_logger().info(
                f'[{timestamp}] SPEED_UP — {old_speed:.2f} -> {self.cruise_speed:.2f} m/s'
            )

        elif command == 'SPEED_DOWN':
            old_speed = self.cruise_speed
            self.cruise_speed = max(
                self.cruise_speed - self.speed_increment, self.min_speed
            )
            self._transition_to(State.DRIVING)
            self.get_logger().info(
                f'[{timestamp}] SPEED_DOWN — {old_speed:.2f} -> {self.cruise_speed:.2f} m/s'
            )

        else:
            self.get_logger().warn(f'[{timestamp}] Unknown command: {command}')

    def _start_turn(self, angular_z: float, duration: float):
        """Begin a timed turn."""
        self.turn_angular_z = angular_z
        self.turn_duration = duration
        self.turn_start_time = time.time()
        self._transition_to(State.TURNING)

    def _transition_to(self, new_state: str):
        """Transition the FSM to a new state."""
        if self.state != new_state:
            timestamp = time.strftime('%H:%M:%S')
            self.get_logger().info(
                f'[{timestamp}] State transition: {self.state} -> {new_state}'
            )
            self.state = new_state
            self._publish_state()

    def _publish_state(self):
        """Publish the current FSM state."""
        state_msg = String()
        state_msg.data = self.state
        self.nav_state_pub.publish(state_msg)

    def _publish_velocity(self, linear_x: float, angular_z: float):
        """Publish a TwistStamped message to /cmd_vel."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_vel_frame_id
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def control_loop(self):
        """Main control loop — runs at control_rate_hz."""
        now = time.time()

        if self.state == State.DRIVING:
            self._publish_velocity(self.cruise_speed, 0.0)

            # Check recovery timeout
            elapsed_since_command = now - self.last_command_time
            if elapsed_since_command >= self.recovery_timeout:
                self._transition_to(State.RECOVERING)
                self.get_logger().info(
                    f'[{time.strftime("%H:%M:%S")}] No command for '
                    f'{self.recovery_timeout}s — entering RECOVERING'
                )

        elif self.state == State.TURNING:
            # Check if turn is complete
            elapsed = now - self.turn_start_time
            if elapsed >= self.turn_duration:
                self.get_logger().info(
                    f'[{time.strftime("%H:%M:%S")}] Turn complete — '
                    f'returning to DRIVING'
                )
                self._transition_to(State.DRIVING)
                self._publish_velocity(self.cruise_speed, 0.0)
            else:
                # Continue turning: no forward motion during turn
                self._publish_velocity(0.0, self.turn_angular_z)

        elif self.state == State.STOPPED:
            self._publish_velocity(0.0, 0.0)

        elif self.state == State.RECOVERING:
            # Slow down to minimum speed
            self._publish_velocity(self.min_speed, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = node.cmd_vel_frame_id
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('Shutting down — stopping robot')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
