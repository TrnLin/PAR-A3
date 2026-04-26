#!/usr/bin/env python3
"""
Signal (traffic light) detection node using HSV color segmentation + depth spatial gating.

Detects RED, YELLOW, GREEN signal cards from camera images and publishes the
current signal state after applying spatial gating and temporal confirmation.
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class SignalDetectorNode(Node):
    """Detects traffic-light signal cards via HSV segmentation and depth gating."""

    def __init__(self):
        super().__init__('signal_detector_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('rgb_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('detection_rate_hz', 15.0)

        # HSV ranges – RED (two ranges because hue wraps)
        self.declare_parameter('red_h_low1', 0)
        self.declare_parameter('red_h_high1', 10)
        self.declare_parameter('red_h_low2', 170)
        self.declare_parameter('red_h_high2', 180)
        self.declare_parameter('red_s_low', 100)
        self.declare_parameter('red_s_high', 255)
        self.declare_parameter('red_v_low', 80)
        self.declare_parameter('red_v_high', 255)

        # HSV – YELLOW
        self.declare_parameter('yellow_h_low', 20)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_low', 100)
        self.declare_parameter('yellow_s_high', 255)
        self.declare_parameter('yellow_v_low', 100)
        self.declare_parameter('yellow_v_high', 255)

        # HSV – GREEN
        self.declare_parameter('green_h_low', 40)
        self.declare_parameter('green_h_high', 80)
        self.declare_parameter('green_s_low', 80)
        self.declare_parameter('green_s_high', 255)
        self.declare_parameter('green_v_low', 80)
        self.declare_parameter('green_v_high', 255)

        # Detection / gating
        self.declare_parameter('min_signal_area', 800)
        self.declare_parameter('max_signal_distance', 1.5)
        self.declare_parameter('gate_angle_deg', 20.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('use_depth_gating', True)

        # Read parameter values
        self._rgb_topic = self.get_parameter('rgb_image_topic').value
        self._depth_topic = self.get_parameter('depth_image_topic').value
        self._rate_hz = self.get_parameter('detection_rate_hz').value

        self._min_area = self.get_parameter('min_signal_area').value
        self._max_dist = self.get_parameter('max_signal_distance').value
        self._gate_angle = self.get_parameter('gate_angle_deg').value
        self._confirm_frames = self.get_parameter('confirm_frames').value
        self._use_depth_gating = self.get_parameter('use_depth_gating').value

        # ── Internal state ──────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_rgb = None
        self._latest_depth = None
        self._confirm_counters = {'RED': 0, 'YELLOW': 0, 'GREEN': 0}
        self._current_state = 'UNKNOWN'

        # ── QoS for sensor topics ───────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            Image, self._rgb_topic, self._rgb_callback, sensor_qos)
        self.create_subscription(
            Image, self._depth_topic, self._depth_callback, sensor_qos)

        # ── Publishers ──────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/signal_state', 10)
        self._detections_pub = self.create_publisher(String, '/signal_detections', 10)

        # ── Detection timer ─────────────────────────────────────────────
        timer_period = 1.0 / self._rate_hz
        self.create_timer(timer_period, self._detect_callback)

        self.get_logger().info(
            f'SignalDetectorNode started – RGB: {self._rgb_topic}, '
            f'Depth: {self._depth_topic}, rate: {self._rate_hz} Hz, '
            f'depth_gating: {self._use_depth_gating}')

    # ── Image callbacks ─────────────────────────────────────────────────

    def _rgb_callback(self, msg: Image):
        try:
            self._latest_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert RGB image: {e}')

    def _depth_callback(self, msg: Image):
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    # ── HSV range helpers ───────────────────────────────────────────────

    def _get_hsv_ranges(self):
        """Return a dict of color -> list of (lower, upper) HSV np arrays."""
        p = self.get_parameter
        return {
            'RED': [
                (np.array([p('red_h_low1').value, p('red_s_low').value, p('red_v_low').value]),
                 np.array([p('red_h_high1').value, p('red_s_high').value, p('red_v_high').value])),
                (np.array([p('red_h_low2').value, p('red_s_low').value, p('red_v_low').value]),
                 np.array([p('red_h_high2').value, p('red_s_high').value, p('red_v_high').value])),
            ],
            'YELLOW': [
                (np.array([p('yellow_h_low').value, p('yellow_s_low').value, p('yellow_v_low').value]),
                 np.array([p('yellow_h_high').value, p('yellow_s_high').value, p('yellow_v_high').value])),
            ],
            'GREEN': [
                (np.array([p('green_h_low').value, p('green_s_low').value, p('green_v_low').value]),
                 np.array([p('green_h_high').value, p('green_s_high').value, p('green_v_high').value])),
            ],
        }

    # ── Core detection logic ────────────────────────────────────────────

    def _detect_callback(self):
        if self._latest_rgb is None:
            return

        rgb = self._latest_rgb
        depth = self._latest_depth  # may be None
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        img_h, img_w = rgb.shape[:2]
        hsv_ranges = self._get_hsv_ranges()

        # Morphological kernels
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # Camera horizontal FOV assumed ~69° for OAK-D Pro
        hfov_deg = 69.0
        cx_image = img_w / 2.0
        fx_approx = (img_w / 2.0) / np.tan(np.radians(hfov_deg / 2.0))

        raw_detections = []  # for JSON logging
        detected_colors = []  # colors that pass all filters this frame

        # Priority order for processing: RED > YELLOW > GREEN
        for color in ['RED', 'YELLOW', 'GREEN']:
            ranges = hsv_ranges[color]

            # Build combined mask
            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            # Morphological clean-up
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # Filter contours
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if h == 0:
                    continue
                aspect = float(w) / float(h)
                if aspect < 0.5 or aspect > 2.0:
                    continue
                valid_contours.append((cnt, area))

            if not valid_contours:
                continue

            # Take the largest valid contour
            valid_contours.sort(key=lambda t: t[1], reverse=True)
            best_cnt, best_area = valid_contours[0]

            # Compute centroid
            M = cv2.moments(best_cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Spatial gating
            gated = True
            det_depth = -1.0

            if self._use_depth_gating and depth is not None:
                # Depth value at centroid
                depth_h, depth_w = depth.shape[:2]
                # Map centroid from RGB to depth coordinate space (assuming aligned)
                dx = int(cx * depth_w / img_w)
                dy = int(cy * depth_h / img_h)
                dx = np.clip(dx, 0, depth_w - 1)
                dy = np.clip(dy, 0, depth_h - 1)

                raw_depth = depth[dy, dx]
                # Depth images may be in millimetres (uint16) or metres (float)
                if depth.dtype == np.uint16:
                    det_depth = float(raw_depth) / 1000.0
                else:
                    det_depth = float(raw_depth)

                # Gate a: distance
                if det_depth <= 0.0 or det_depth > self._max_dist:
                    gated = False

                # Gate b: angular position relative to image centre
                angle_deg = np.degrees(np.arctan2(cx - cx_image, fx_approx))
                if abs(angle_deg) > self._gate_angle:
                    gated = False

            elif self._use_depth_gating and depth is None:
                # Depth not available yet – don't gate but note it
                gated = True
                det_depth = -1.0

            raw_detections.append({
                'color': color,
                'area': int(best_area),
                'depth': round(det_depth, 3),
                'gated': gated,
                'centroid': [cx, cy],
            })

            if gated:
                detected_colors.append(color)

        # ── Temporal confirmation ───────────────────────────────────────
        # Increment counters for detected, reset for not-detected
        for color in ['RED', 'YELLOW', 'GREEN']:
            if color in detected_colors:
                self._confirm_counters[color] += 1
            else:
                self._confirm_counters[color] = 0

        # Determine confirmed colors (met frame threshold)
        confirmed_colors = [
            c for c in ['RED', 'YELLOW', 'GREEN']
            if self._confirm_counters[c] >= self._confirm_frames
        ]

        # Priority: RED > YELLOW > GREEN
        if 'RED' in confirmed_colors:
            new_state = 'RED'
        elif 'YELLOW' in confirmed_colors:
            new_state = 'YELLOW'
        elif 'GREEN' in confirmed_colors:
            new_state = 'GREEN'
        else:
            new_state = 'UNKNOWN'

        if new_state != self._current_state:
            self.get_logger().info(
                f'Signal state transition: {self._current_state} -> {new_state}')
            self._current_state = new_state

        # ── Publish ─────────────────────────────────────────────────────
        state_msg = String()
        state_msg.data = self._current_state
        self._state_pub.publish(state_msg)

        det_msg = String()
        det_msg.data = json.dumps({
            'timestamp': time.time(),
            'detections': raw_detections,
            'confirmed_state': self._current_state,
        })
        self._detections_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SignalDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
