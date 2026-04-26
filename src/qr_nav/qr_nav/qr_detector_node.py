#!/usr/bin/env python3
"""QR Code Detector Node for ROSbot 3 PRO with OAK-D Pro camera.

Detects and decodes QR codes from camera images, validates commands,
and publishes the highest-priority command based on bounding box area.
"""

import json
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


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


class QRDetectorNode(Node):
    """Detects QR codes from the OAK-D Pro camera and publishes validated commands."""

    def __init__(self):
        super().__init__('qr_detector_node')

        # Declare parameters
        self.declare_parameter('rgb_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('detection_rate_hz', 15.0)
        self.declare_parameter('min_bbox_area', 500)

        # Read parameters
        rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        detection_rate_hz = self.get_parameter('detection_rate_hz').get_parameter_value().double_value
        self.min_bbox_area = self.get_parameter('min_bbox_area').get_parameter_value().integer_value

        # CV bridge and QR detector
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # Cooldown tracking: command -> last publish timestamp
        self.last_command_time = {}
        self.cooldown_sec = 1.0

        # Rate limiting: only process frames at detection_rate_hz
        self.detection_period = 1.0 / detection_rate_hz
        self.last_detection_time = 0.0

        # QoS for OAK-D Pro camera: BEST_EFFORT reliability, VOLATILE durability
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.image_callback,
            camera_qos,
        )

        # Publishers
        self.command_pub = self.create_publisher(String, '/qr_command', 10)
        self.detections_pub = self.create_publisher(String, '/qr_detections', 10)

        self.get_logger().info(
            f'QR Detector Node started — topic: {rgb_topic}, '
            f'rate: {detection_rate_hz} Hz, min_bbox_area: {self.min_bbox_area}'
        )

    def image_callback(self, msg: Image):
        """Process incoming camera image for QR codes."""
        now = time.time()

        # Rate limit detection
        if now - self.last_detection_time < self.detection_period:
            return
        self.last_detection_time = now

        # Convert ROS Image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Preprocess: grayscale + CLAHE for contrast enhancement
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
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

            # Compute bounding box area from the corner points
            bbox_points = points[i]
            bbox_area = cv2.contourArea(bbox_points.astype(float))

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
