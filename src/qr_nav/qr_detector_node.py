"""QR Code Detector Node for ROSbot 3 PRO with OAK-D Pro camera.

Detects and decodes QR codes from camera images, validates commands,
and publishes the highest-priority command based on bounding box area.

Also runs a passive lighting monitor (Pass 1 of the adaptive-lighting
work): frame luma is tracked as an EMA, decode successes are tracked in
a sliding window, and a candidate lighting state (BRIGHT / DIM / DARK)
is computed with hysteresis + dwell. Pass 1 only publishes the
classification; no sensor or IR change is made yet. Later passes hook
this signal into a switch handshake with the command interpreter.
"""

import collections
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# AsyncParameterClient was introduced in rclpy for ROS 2 Iron. The
# qr_detector node is fully functional without IR control (Pass 1 + 2),
# so import lazily and fall back to a no-op if it isn't available on the
# robot's rclpy build.
try:
    from rclpy.parameter_client import AsyncParameterClient  # type: ignore
    _IR_CLIENT_AVAILABLE = True
except Exception:
    AsyncParameterClient = None  # type: ignore[assignment]
    _IR_CLIENT_AVAILABLE = False


# Valid QR command set
VALID_COMMANDS = {
    'TURN_LEFT',
    'TURN_RIGHT',
    'STOP',
    'GO',
    'SPEED_UP',
    'SPEED_DOWN',
    'U_TURN',
}


# Lighting state constants. Order matters: BRIGHT < DIM < DARK is used
# as an implicit "darkness rank" in the classifier below.
LIGHTING_BRIGHT = 'BRIGHT'
LIGHTING_DIM = 'DIM'
LIGHTING_DARK = 'DARK'
LIGHTING_STATES = (LIGHTING_BRIGHT, LIGHTING_DIM, LIGHTING_DARK)


# Sensor mode constants used in the switch handshake.
SENSOR_RGB = 'RGB'
SENSOR_MONO = 'MONO'


# Handshake sub-state. INTERNAL to the detector — the interpreter only
# ever sees /qr_detector/switch_request and /qr_detector/switch_complete.
SWITCH_IDLE = 'IDLE'                # no switch in flight
SWITCH_REQUESTED = 'REQUESTED'      # waiting for interpreter to reach STOPPED/IDLE
SWITCH_VALIDATING = 'VALIDATING'    # new sensor active, awaiting first valid decode


# Command interpreter FSM state names we care about. Kept as bare strings
# rather than importing because qr_nav has no internal package boundary
# yet and we want the detector to remain runnable standalone.
FSM_IDLE = 'IDLE'
FSM_STOPPED = 'STOPPED'


class QRDetectorNode(Node):
    """Detects QR codes from the OAK-D Pro camera and publishes validated commands."""

    def __init__(self):
        super().__init__('qr_detector_node')

        # Declare parameters
        # Default topic = ROSbot 3 PRO / Husarion OAK-D namespace.
        self.declare_parameter('rgb_image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('detection_rate_hz', 15.0)
        self.declare_parameter('min_bbox_area', 500)
        # Camera QoS reliability must match the publisher, otherwise the
        # subscriber silently receives zero frames. Husarion's OAK-D ships
        # RELIABLE; some other depthai launches use BEST_EFFORT.
        self.declare_parameter('image_qos_reliability', 'reliable')

        # --- Adaptive-lighting observability (Pass 1) ---
        # Pass 1 only OBSERVES — no sensor switch and no IR control yet.
        # Defaults are conservative starting points; tune from CSV data
        # collected in the demo room (see Pass 4 of the plan).
        self.declare_parameter('adaptive_lighting', True)
        self.declare_parameter('mono_image_topic', '/oak/left/image_raw')
        self.declare_parameter('luma_dim_threshold', 60.0)
        self.declare_parameter('luma_dark_threshold', 25.0)
        self.declare_parameter('luma_bright_hysteresis', 20.0)
        self.declare_parameter('luma_ema_alpha', 0.1)
        self.declare_parameter('state_dwell_sec', 2.0)
        self.declare_parameter('decode_window_frames', 30)
        self.declare_parameter('decode_failure_threshold', 0.0)
        self.declare_parameter('lighting_publish_rate_hz', 2.0)
        # Pass 2: maximum time (s) to wait for a valid QR decode on the
        # NEW sensor after a switch. Timing out leaves the robot STOPPED
        # and the operator must intervene with a GO card.
        self.declare_parameter('validation_timeout_sec', 10.0)
        # On validation timeout, roll back to the RGB feed so the lighting
        # monitor can recover (the alternative is being stranded on a feed
        # whose luma_ema can't update — e.g. mono cam not publishing, or
        # an undecodable IR-illuminated scene). After a failure, gate
        # retries to the same target sensor for this many seconds so the
        # detector doesn't ping-pong between sensors when the underlying
        # condition (no mono frames, undecodable QR under IR, etc.) hasn't
        # changed. Set to 0 to disable the cooldown.
        self.declare_parameter('switch_failure_cooldown_sec', 30.0)

        # Pass 3: IR floodlight + dot-projector control via the depthai
        # camera node's parameter interface. Names ship varying across
        # depthai-ros versions, so they are configurable via yaml.
        self.declare_parameter('depthai_node_name', '/oak')
        self.declare_parameter('floodlight_param_name', 'i_floodlight_brightness')
        self.declare_parameter('dot_projector_param_name', 'i_laser_dot_projector_current')
        # Optional master IR toggle. Husarion's depthai-ros gates the
        # floodlight current on `camera.i_enable_ir` — without flipping it,
        # writing the floodlight is silently a no-op. Empty string disables
        # this behaviour for upstream depthai-ros builds that don't expose
        # a toggle (the floodlight param is unconditional there).
        self.declare_parameter('ir_enable_param_name', '')
        # Floodlight current (mA) per confirmed lighting state. The OAK-D
        # Pro accepts roughly 0..1500 mA; values listed here are safe
        # starting points and should be re-tuned per Pass 4 in the demo
        # room. The dot projector is ALWAYS forced to 0 — it puts a
        # speckle pattern over the scene that destroys QR decoding.
        self.declare_parameter('ir_floodlight_bright_mA', 0)
        self.declare_parameter('ir_floodlight_dim_mA', 300)
        self.declare_parameter('ir_floodlight_dark_mA', 1000)

        # Read parameters
        rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        detection_rate_hz = self.get_parameter('detection_rate_hz').get_parameter_value().double_value
        self.min_bbox_area = self.get_parameter('min_bbox_area').get_parameter_value().integer_value
        qos_reliability_str = (
            self.get_parameter('image_qos_reliability').get_parameter_value().string_value
        ).strip().lower()

        # Lighting-monitor params
        self.adaptive_lighting = (
            self.get_parameter('adaptive_lighting').get_parameter_value().bool_value
        )
        self.mono_image_topic = (
            self.get_parameter('mono_image_topic').get_parameter_value().string_value
        )
        self.luma_dim_threshold = (
            self.get_parameter('luma_dim_threshold').get_parameter_value().double_value
        )
        self.luma_dark_threshold = (
            self.get_parameter('luma_dark_threshold').get_parameter_value().double_value
        )
        self.luma_bright_hysteresis = (
            self.get_parameter('luma_bright_hysteresis').get_parameter_value().double_value
        )
        self.luma_ema_alpha = (
            self.get_parameter('luma_ema_alpha').get_parameter_value().double_value
        )
        self.state_dwell_sec = (
            self.get_parameter('state_dwell_sec').get_parameter_value().double_value
        )
        decode_window_frames = (
            self.get_parameter('decode_window_frames').get_parameter_value().integer_value
        )
        self.decode_failure_threshold = (
            self.get_parameter('decode_failure_threshold').get_parameter_value().double_value
        )
        lighting_publish_rate_hz = (
            self.get_parameter('lighting_publish_rate_hz').get_parameter_value().double_value
        )
        self.validation_timeout_sec = (
            self.get_parameter('validation_timeout_sec').get_parameter_value().double_value
        )
        self.switch_failure_cooldown_sec = (
            self.get_parameter('switch_failure_cooldown_sec').get_parameter_value().double_value
        )

        # IR control params
        self.depthai_node_name = (
            self.get_parameter('depthai_node_name').get_parameter_value().string_value
        )
        self.floodlight_param_name = (
            self.get_parameter('floodlight_param_name').get_parameter_value().string_value
        )
        self.dot_projector_param_name = (
            self.get_parameter('dot_projector_param_name').get_parameter_value().string_value
        )
        self.ir_enable_param_name = (
            self.get_parameter('ir_enable_param_name').get_parameter_value().string_value
        )
        self.ir_floodlight_mA = {
            LIGHTING_BRIGHT: int(
                self.get_parameter('ir_floodlight_bright_mA').get_parameter_value().integer_value
            ),
            LIGHTING_DIM: int(
                self.get_parameter('ir_floodlight_dim_mA').get_parameter_value().integer_value
            ),
            LIGHTING_DARK: int(
                self.get_parameter('ir_floodlight_dark_mA').get_parameter_value().integer_value
            ),
        }

        # CV bridge and QR detector
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # Cooldown tracking: command -> last publish timestamp
        self.last_command_time = {}
        self.cooldown_sec = 1.0

        # Rate limiting: only process frames at detection_rate_hz
        self.detection_period = 1.0 / detection_rate_hz
        self.last_detection_time = 0.0

        # --- Lighting monitor state ---
        # luma_ema seeds at mid-grey so a single dim startup frame doesn't
        # immediately drag us into DIM/DARK on the first sample.
        self.luma_ema = 128.0
        # Per-frame decode success (True if at least one decoded payload).
        self.decode_window = collections.deque(maxlen=max(decode_window_frames, 1))
        # Confirmed (= published) lighting state and the rolling candidate
        # we're "auditioning". A transition only fires once the candidate
        # has been stable for `state_dwell_sec`.
        self.lighting_state = LIGHTING_BRIGHT
        self.candidate_state = LIGHTING_BRIGHT
        self.candidate_since = time.time()

        # Camera QoS — reliability is parameterised (image_qos_reliability)
        # because it MUST match the publisher or the subscriber gets zero
        # callbacks. Husarion's OAK-D ships RELIABLE / VOLATILE (verified
        # with `ros2 topic info /oak/rgb/image_raw -v`); other depthai
        # launches sometimes publish BEST_EFFORT.
        if qos_reliability_str == 'best_effort':
            reliability = ReliabilityPolicy.BEST_EFFORT
        elif qos_reliability_str == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            self.get_logger().warn(
                f'Unknown image_qos_reliability={qos_reliability_str!r}; '
                f'defaulting to RELIABLE.'
            )
            reliability = ReliabilityPolicy.RELIABLE

        # Persist QoS + topic names so the switch handshake can rebuild
        # the subscription on a different topic at runtime (Pass 2).
        self.camera_qos = QoSProfile(
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb_image_topic = rgb_topic

        # Active sensor — bgr8 for RGB, mono8 for the OAK-D mono cameras.
        self.current_sensor_mode = SENSOR_RGB

        # Handshake state for the switch protocol (Pass 2). Driven from
        # the lighting timer; mutated by transitions in the image callback
        # (on valid detections) and by /nav_state updates.
        self.switch_phase = SWITCH_IDLE
        self.switch_target_sensor = None
        self.switch_target_lighting = None
        self.validation_start_time = 0.0
        # Track the most recent failed switch target so the cooldown gate
        # in `_drive_switch_handshake` can suppress immediate retries.
        # `None` means "no failure on record".
        self.last_switch_failure_time = 0.0
        self.last_switch_failure_target = None
        # Latest FSM state heard from the command interpreter on /nav_state.
        # Stays None until the interpreter sends its first state — that's
        # OK; in IDLE/STOPPED-only gating we just wait.
        self.last_nav_state = None

        # Subscriber — destroyed and re-created when sensor mode changes.
        self.image_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.image_callback,
            self.camera_qos,
        )

        # /nav_state subscriber: the detector watches the interpreter's
        # FSM so it knows when it's safe to actually perform a switch.
        self.nav_state_sub = self.create_subscription(
            String,
            '/nav_state',
            self._nav_state_callback,
            10,
        )

        # Publishers
        self.command_pub = self.create_publisher(String, '/qr_command', 10)
        self.detections_pub = self.create_publisher(String, '/qr_detections', 10)
        # Lighting-monitor outputs: a plain-text state name for cheap
        # subscribers (data_logger, future interpreter handshake), plus a
        # JSON metrics topic for `ros2 topic echo` during tuning.
        self.lighting_state_pub = self.create_publisher(
            String, '/qr_detector/lighting_state', 10
        )
        self.lighting_metrics_pub = self.create_publisher(
            String, '/qr_detector/lighting_metrics', 10
        )

        # Switch-handshake topics (Pass 2). The interpreter subscribes to
        # both; the detector publishes the request then waits for the FSM
        # to reach IDLE/STOPPED, performs the sensor swap, and finally
        # publishes "ok" once a valid QR is decoded on the new sensor.
        self.switch_request_pub = self.create_publisher(
            String, '/qr_detector/switch_request', 10
        )
        self.switch_complete_pub = self.create_publisher(
            String, '/qr_detector/switch_complete', 10
        )

        # Periodic lighting publish + handshake-tick timer. Independent
        # of camera frame rate so the handshake can still progress (and
        # subscribers still hear from us) even if the camera stalls
        # mid-switch.
        lighting_period = 1.0 / max(lighting_publish_rate_hz, 0.1)
        self.lighting_timer = self.create_timer(
            lighting_period, self._lighting_tick
        )

        # Pass 3: IR controller. Best-effort — if the depthai node is not
        # up yet or AsyncParameterClient is unavailable, we log and
        # continue; the IR set will be retried on every lighting state
        # transition and on every sensor switch completion.
        self.ir_client = None
        if self.adaptive_lighting and _IR_CLIENT_AVAILABLE:
            try:
                self.ir_client = AsyncParameterClient(self, self.depthai_node_name)
                self.get_logger().info(
                    f'IR parameter client targeting {self.depthai_node_name} '
                    f'(floodlight={self.floodlight_param_name}, '
                    f'projector={self.dot_projector_param_name})'
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Could not build AsyncParameterClient for '
                    f'{self.depthai_node_name}: {e}. IR control disabled.'
                )
        elif self.adaptive_lighting and not _IR_CLIENT_AVAILABLE:
            self.get_logger().warn(
                'rclpy.parameter_client.AsyncParameterClient is unavailable; '
                'IR control disabled. Lighting state will still be published.'
            )

        # Apply the initial-state IR profile (BRIGHT defaults). Fire and
        # forget; depthai may not be up yet — retried on every transition.
        self._apply_ir_for_state(self.lighting_state)

        self.get_logger().info(
            f'QR Detector Node started — topic: {rgb_topic}, '
            f'rate: {detection_rate_hz} Hz, min_bbox_area: {self.min_bbox_area}, '
            f'qos: {reliability.name}, '
            f'adaptive_lighting: {self.adaptive_lighting} '
            f'(mono_topic={self.mono_image_topic}, '
            f'validation_timeout={self.validation_timeout_sec}s)'
        )

    def _nav_state_callback(self, msg: String):
        """Track the command-interpreter FSM state for handshake gating."""
        self.last_nav_state = msg.data

    def image_callback(self, msg: Image):
        """Process incoming camera image for QR codes."""
        now = time.time()

        # Rate limit detection
        if now - self.last_detection_time < self.detection_period:
            return
        self.last_detection_time = now

        # Pick the cv_bridge encoding for the active sensor. The OAK-D
        # mono streams arrive as 8-bit single-channel — converting to
        # bgr8 would still work but wastes a memcpy on every frame.
        desired_encoding = (
            'mono8' if self.current_sensor_mode == SENSOR_MONO else 'bgr8'
        )
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding=desired_encoding
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Grayscale conversion is a no-op on mono8 frames.
        if self.current_sensor_mode == SENSOR_MONO:
            gray = cv_image
        else:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Preprocess: CLAHE for contrast enhancement
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        # Attempt QR detection with enhanced image
        detections = self._detect_qr_codes(enhanced)

        # Fallback: adaptive thresholding for degraded/partial codes
        if not detections:
            thresh = cv2.adaptiveThreshold(
                enhanced, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                blockSize=11,
                C=2,
            )
            detections = self._detect_qr_codes(thresh)

        # Feed the lighting monitor BEFORE we early-return on empty
        # detections — "no detections this frame" is itself a signal we
        # want in the decode-success window.
        self._update_lighting_monitor(gray, bool(detections))

        if not detections:
            return

        # Publish all detections as JSON for logging
        timestamp_str = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now))
        all_detections = []
        valid_detections = []

        for content, bbox_area in detections:
            entry = {
                'timestamp': timestamp_str,
                'content': content,
                'bbox_area': bbox_area,
            }
            all_detections.append(entry)

            # Validate command and enforce min bbox area
            if content in VALID_COMMANDS and bbox_area >= self.min_bbox_area:
                valid_detections.append((content, bbox_area))

        # Publish all detections (including invalid) for logging
        det_msg = String()
        det_msg.data = json.dumps(all_detections)
        self.detections_pub.publish(det_msg)

        # Pass 2: a valid decode on the post-switch sensor proves the
        # switch worked. Complete validation BEFORE the cooldown check so
        # we don't sit in VALIDATING just because the same QR fired twice.
        if self.switch_phase == SWITCH_VALIDATING and valid_detections:
            self._complete_switch('ok')

        if not valid_detections:
            return

        # Pick the detection with the largest bounding box area (closest QR code)
        best_command, best_area = max(valid_detections, key=lambda x: x[1])

        # Cooldown check: avoid re-triggering the same command within cooldown period
        last_time = self.last_command_time.get(best_command, 0.0)
        if now - last_time < self.cooldown_sec:
            return

        # Publish the validated command
        cmd_msg = String()
        cmd_msg.data = best_command
        self.command_pub.publish(cmd_msg)
        self.last_command_time[best_command] = now

        self.get_logger().info(
            f'QR command published: {best_command} (bbox_area={best_area})'
        )

    def _update_lighting_monitor(self, gray_frame, decoded_this_frame: bool):
        """Update luma EMA, decode-success window, and the lighting state.

        Pass 1 behaviour: observation only. The confirmed `lighting_state`
        is published on a timer; no sensor or IR control is touched.
        Later passes hook a switch handshake onto state transitions.
        """
        if not self.adaptive_lighting:
            return

        try:
            frame_luma = float(gray_frame.mean())
        except Exception as e:
            self.get_logger().debug(f'luma compute failed: {e}')
            return

        alpha = self.luma_ema_alpha
        self.luma_ema = alpha * frame_luma + (1.0 - alpha) * self.luma_ema
        self.decode_window.append(decoded_this_frame)

        decode_rate = (
            sum(1 for v in self.decode_window if v) / len(self.decode_window)
            if self.decode_window else 0.0
        )
        window_full = len(self.decode_window) == self.decode_window.maxlen

        candidate = self._classify_lighting(
            self.luma_ema, decode_rate, window_full
        )

        now = time.time()
        if candidate != self.candidate_state:
            # Candidate flipped — reset the dwell timer. We don't transition
            # yet; the candidate must remain stable for `state_dwell_sec`.
            self.candidate_state = candidate
            self.candidate_since = now
            return

        # Candidate is stable. If it disagrees with the confirmed state and
        # has been stable long enough, promote it.
        if (
            candidate != self.lighting_state
            and now - self.candidate_since >= self.state_dwell_sec
        ):
            prev = self.lighting_state
            self.lighting_state = candidate
            self.get_logger().info(
                f'Lighting state: {prev} -> {candidate} '
                f'(luma_ema={self.luma_ema:.1f}, decode_rate={decode_rate:.2f})'
            )
            # Pass 3: push the IR profile to the depthai node. Floodlight
            # is invisible to the (IR-cut-filtered) RGB sensor, so it's
            # safe to enable BEFORE the sensor switch — the new mono
            # sensor sees an already-illuminated scene the moment we
            # re-subscribe.
            self._apply_ir_for_state(self.lighting_state)

    def _classify_lighting(self, luma: float, decode_rate: float, window_full: bool) -> str:
        """Pick a candidate lighting state from the current observations.

        Hysteresis: each state has a luma threshold to ENTER; you have to
        rise `luma_bright_hysteresis` ABOVE that threshold before falling
        back out. This stops oscillation at the boundary.

        Decode-rate escalation: if the rolling window is full and decode
        rate is at the configured floor (default 0.0 = zero decodes for
        the entire window), force one step darker as long as luma is near
        a transition boundary. Catches scenes where raw luma looks fine
        but contrast/glare has killed decoding.
        """
        upper_dim = self.luma_dim_threshold + self.luma_bright_hysteresis
        upper_dark = self.luma_dark_threshold + self.luma_bright_hysteresis

        if self.lighting_state == LIGHTING_BRIGHT:
            candidate = LIGHTING_DIM if luma < self.luma_dim_threshold else LIGHTING_BRIGHT
        elif self.lighting_state == LIGHTING_DIM:
            if luma < self.luma_dark_threshold:
                candidate = LIGHTING_DARK
            elif luma > upper_dim:
                candidate = LIGHTING_BRIGHT
            else:
                candidate = LIGHTING_DIM
        else:  # LIGHTING_DARK
            candidate = LIGHTING_DIM if luma > upper_dark else LIGHTING_DARK

        # Decode-rate-driven escalation: only fires when the window is
        # full (avoid noisy startup) and the candidate did not already
        # propose a darker state on its own.
        if (
            window_full
            and decode_rate <= self.decode_failure_threshold
            and candidate == self.lighting_state
        ):
            if self.lighting_state == LIGHTING_BRIGHT and luma < upper_dim:
                candidate = LIGHTING_DIM
            elif self.lighting_state == LIGHTING_DIM and luma < upper_dark:
                candidate = LIGHTING_DARK

        return candidate

    def _lighting_tick(self):
        """Publish lighting telemetry and drive the switch handshake."""
        self._publish_lighting_state()
        if self.adaptive_lighting:
            self._drive_switch_handshake()

    def _publish_lighting_state(self):
        """Publish the current lighting state + metrics."""
        state_msg = String()
        state_msg.data = self.lighting_state
        self.lighting_state_pub.publish(state_msg)

        decode_rate = (
            sum(1 for v in self.decode_window if v) / len(self.decode_window)
            if self.decode_window else 0.0
        )
        metrics_msg = String()
        metrics_msg.data = json.dumps({
            'state': self.lighting_state,
            'candidate': self.candidate_state,
            'luma_ema': round(self.luma_ema, 2),
            'decode_rate': round(decode_rate, 3),
            'decode_window_filled': len(self.decode_window),
            'decode_window_max': self.decode_window.maxlen,
            'adaptive_lighting_enabled': self.adaptive_lighting,
            'sensor_mode': self.current_sensor_mode,
            'switch_phase': self.switch_phase,
            'switch_target_sensor': self.switch_target_sensor,
            'nav_state': self.last_nav_state,
        })
        self.lighting_metrics_pub.publish(metrics_msg)

    # ------------------------------------------------------------------
    # Switch handshake (Pass 2)
    # ------------------------------------------------------------------

    def _sensor_mode_for_lighting(self, lighting_state: str) -> str:
        """RGB for BRIGHT/DIM (Pass 2 default), MONO for DARK."""
        return SENSOR_MONO if lighting_state == LIGHTING_DARK else SENSOR_RGB

    def _drive_switch_handshake(self):
        """Advance the switch handshake state machine.

        Idle: open a new switch if the confirmed lighting state demands a
        sensor mode change.
        Requested: wait for the interpreter to reach IDLE/STOPPED, then
        perform the physical switch.
        Validating: enforce the validation timeout. Success is signalled
        from `image_callback` when the first valid decode arrives.
        """
        now = time.time()

        if self.switch_phase == SWITCH_IDLE:
            desired_sensor = self._sensor_mode_for_lighting(self.lighting_state)
            if desired_sensor != self.current_sensor_mode:
                # Cooldown after a failed validation: don't immediately
                # re-request the same target sensor. Without this we
                # ping-pong between RGB and MONO whenever DARK is
                # confirmed but the mono path can't decode (no frames,
                # phone screen under IR, etc.). Operator must let the
                # cooldown elapse OR change conditions (lights on,
                # printed card) for the next attempt.
                if (
                    self.last_switch_failure_target is not None
                    and desired_sensor == self.last_switch_failure_target
                    and now - self.last_switch_failure_time
                    < self.switch_failure_cooldown_sec
                ):
                    return
                self._request_switch(desired_sensor)
            return

        if self.switch_phase == SWITCH_REQUESTED:
            if self.last_nav_state in (FSM_IDLE, FSM_STOPPED):
                self._perform_switch()
            return

        if self.switch_phase == SWITCH_VALIDATING:
            if now - self.validation_start_time >= self.validation_timeout_sec:
                self._complete_switch('timeout')
            return

    def _request_switch(self, target_sensor: str):
        """Ask the interpreter to bring the robot to STOPPED for a switch."""
        self.switch_phase = SWITCH_REQUESTED
        self.switch_target_sensor = target_sensor
        self.switch_target_lighting = self.lighting_state

        msg = String()
        # Message payload is the target lighting state name — that's the
        # most useful thing for human operators tailing the topic.
        msg.data = self.lighting_state
        self.switch_request_pub.publish(msg)
        self.get_logger().info(
            f'Switch requested: lighting={self.lighting_state}, '
            f'sensor {self.current_sensor_mode} -> {target_sensor}; '
            f'awaiting interpreter to reach IDLE/STOPPED '
            f'(current nav_state={self.last_nav_state})'
        )

    def _perform_switch(self):
        """Tear down and re-subscribe on the new sensor topic."""
        target = self.switch_target_sensor
        if target is None:
            self.get_logger().warn(
                'Switch tried to perform with no target sensor; aborting'
            )
            self.switch_phase = SWITCH_IDLE
            return

        topic = (
            self.mono_image_topic if target == SENSOR_MONO else self.rgb_image_topic
        )
        self.get_logger().info(
            f'Performing switch: subscribing to {topic} as {target}'
        )
        try:
            self.destroy_subscription(self.image_sub)
        except Exception as e:
            self.get_logger().warn(f'destroy_subscription failed: {e}')
        self.image_sub = self.create_subscription(
            Image, topic, self.image_callback, self.camera_qos
        )
        self.current_sensor_mode = target
        self.switch_phase = SWITCH_VALIDATING
        self.validation_start_time = time.time()
        # Reset rolling decode window so old-sensor history doesn't
        # immediately trigger another lighting transition on the new feed.
        self.decode_window.clear()

    def _complete_switch(self, result: str):
        """Finalise the switch and notify the interpreter."""
        msg = String()
        msg.data = result
        self.switch_complete_pub.publish(msg)

        if result == 'ok':
            self.get_logger().info(
                f'Switch validated on {self.current_sensor_mode} '
                f'(lighting={self.lighting_state})'
            )
            # Re-apply IR for the confirmed state. If the very first
            # apply on transition failed (depthai not up yet) this is a
            # second chance to set the correct floodlight current.
            self._apply_ir_for_state(self.lighting_state)
            # Successful validation clears any prior failure cooldown so
            # the opposite-direction switch can fire freely on recovery.
            self.last_switch_failure_target = None
        else:
            failed_sensor = self.current_sensor_mode
            self.get_logger().warn(
                f'Switch validation timed out on {failed_sensor} '
                f'after {self.validation_timeout_sec:.1f}s — reverting '
                f'to RGB; robot stays STOPPED until operator sends GO. '
                f'Common causes: mono topic not publishing '
                f'({self.mono_image_topic}), or QR target not visible '
                f'under IR (phone screens emit no IR — use a printed card).'
            )
            self.last_switch_failure_time = time.time()
            self.last_switch_failure_target = failed_sensor
            # Roll back so the lighting monitor isn't stranded on a feed
            # it can't recover from. The cooldown above prevents an
            # immediate retry to the same (failed) sensor.
            self._revert_to_rgb()

        self.switch_phase = SWITCH_IDLE
        self.switch_target_sensor = None
        self.switch_target_lighting = None

    def _revert_to_rgb(self):
        """Fail-safe rollback to the RGB feed after a validation timeout.

        Without this the detector stays subscribed to a feed it can't
        recover from (no frames, undecodable QR under IR, etc.), the
        lighting monitor's luma_ema can never update, and the only way
        out is to kill and relaunch the node. Reverting also resets the
        lighting state so the monitor re-observes the room from BRIGHT
        and decides fresh whether it's dark enough to retry.
        """
        if self.current_sensor_mode == SENSOR_RGB:
            return
        self.get_logger().info(
            f'Reverting subscription: {self.current_sensor_mode} -> RGB '
            f'on {self.rgb_image_topic}'
        )
        try:
            self.destroy_subscription(self.image_sub)
        except Exception as e:
            self.get_logger().warn(f'destroy_subscription on revert failed: {e}')
        self.image_sub = self.create_subscription(
            Image, self.rgb_image_topic, self.image_callback, self.camera_qos
        )
        self.current_sensor_mode = SENSOR_RGB
        # Reset the lighting monitor so it re-observes from scratch on
        # the (now restored) RGB feed. Seeding luma_ema at mid-grey
        # avoids the carryover stale-dark value triggering an immediate
        # re-transition to DARK before any RGB frames are processed.
        self.lighting_state = LIGHTING_BRIGHT
        self.candidate_state = LIGHTING_BRIGHT
        self.candidate_since = time.time()
        self.decode_window.clear()
        self.luma_ema = 128.0
        # Turn IR off explicitly. _apply_ir_for_state(BRIGHT) writes
        # floodlight=0 mA which (with the master toggle) also flips
        # camera.i_enable_ir back to false.
        self._apply_ir_for_state(LIGHTING_BRIGHT)

    # ------------------------------------------------------------------
    # IR floodlight + dot-projector control (Pass 3)
    # ------------------------------------------------------------------

    def _apply_ir_for_state(self, lighting_state: str):
        """Apply the floodlight current for `lighting_state`, projector 0.

        Best-effort: failures are logged and ignored. The next confirmed
        lighting transition (or sensor switch completion) re-applies, so
        a late-starting depthai node eventually catches up.
        """
        if self.ir_client is None:
            return
        floodlight_mA = self.ir_floodlight_mA.get(lighting_state, 0)
        # Projector is ALWAYS off for QR work — its speckle dots are
        # designed to add texture for stereo depth, which is exactly the
        # kind of high-frequency noise that wrecks QR decoding.
        self._set_ir_parameters(floodlight_mA=floodlight_mA, projector_mA=0)

    def _set_ir_parameters(self, floodlight_mA: int, projector_mA: int):
        """Push floodlight + projector currents to the depthai node.

        Returns immediately; result is logged from the future's
        done-callback so the detector stays non-blocking.
        """
        if self.ir_client is None:
            return
        try:
            params = []
            # Some depthai-ros builds gate the floodlight current on a
            # separate bool master toggle (e.g. Husarion's
            # `camera.i_enable_ir`). When configured, push it atomically
            # with the currents so the driver doesn't ignore the writes.
            # Enable iff at least one IR emitter is requested non-zero.
            if self.ir_enable_param_name:
                params.append(
                    Parameter(
                        self.ir_enable_param_name,
                        Parameter.Type.BOOL,
                        bool(floodlight_mA > 0 or projector_mA > 0),
                    ).to_parameter_msg()
                )
            params.extend([
                Parameter(
                    self.floodlight_param_name,
                    Parameter.Type.INTEGER,
                    int(floodlight_mA),
                ).to_parameter_msg(),
                Parameter(
                    self.dot_projector_param_name,
                    Parameter.Type.INTEGER,
                    int(projector_mA),
                ).to_parameter_msg(),
            ])
            future = self.ir_client.set_parameters(params)
            future.add_done_callback(
                lambda f: self._log_ir_result(f, floodlight_mA, projector_mA)
            )
        except Exception as e:
            self.get_logger().warn(
                f'IR parameter set raised: {e} '
                f'(floodlight={floodlight_mA}, projector={projector_mA})'
            )

    def _log_ir_result(self, future, floodlight_mA: int, projector_mA: int):
        """Done-callback for the parameter-set future."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(
                f'IR parameter set failed (depthai down?): {e}. '
                f'Will retry on next lighting transition.'
            )
            return
        results = getattr(response, 'results', None) or []
        ok = all(getattr(r, 'successful', False) for r in results)
        if ok:
            self.get_logger().info(
                f'IR applied: floodlight={floodlight_mA} mA, '
                f'projector={projector_mA} mA'
            )
        else:
            reasons = [
                getattr(r, 'reason', '') for r in results
                if not getattr(r, 'successful', False)
            ]
            self.get_logger().warn(
                f'IR partial/failed: floodlight={floodlight_mA}, '
                f'projector={projector_mA}, reasons={reasons!r}. '
                f'Verify {self.floodlight_param_name} / '
                f'{self.dot_projector_param_name} on {self.depthai_node_name}.'
            )

    def _detect_qr_codes(self, image):
        """Detect and decode QR codes in the given image.

        Returns a list of (decoded_text, bbox_area) tuples.
        """
        detections = []
        try:
            retval, decoded_info, points, straight_qrcode = (
                self.qr_detector.detectAndDecodeMulti(image)
            )
        except Exception as e:
            self.get_logger().debug(f'QR detectAndDecodeMulti failed: {e}')
            return detections

        if not retval or decoded_info is None or points is None:
            return detections

        for i, content in enumerate(decoded_info):
            if not content:
                continue

            # Compute bounding box area from the corner points.
            # cv2.contourArea on OpenCV 4.6 (ROS 2 Jazzy default) requires
            # CV_32F or CV_32S — np.float64 (the default for .astype(float))
            # raises (-215:Assertion failed) depth == CV_32F || CV_32S.
            # Wrap in try/except so a single malformed detection becomes a
            # warning instead of crashing the node (and stranding the robot).
            try:
                bbox_points = np.asarray(points[i], dtype=np.float32)
                bbox_area = cv2.contourArea(bbox_points)
            except Exception as e:
                self.get_logger().warn(
                    f'Skipping detection {i!r}={content!r}: bbox area calc '
                    f'failed ({e})'
                )
                continue

            detections.append((content.strip(), bbox_area))

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
