"""Publish one stable QR result, then stop capture and release the camera."""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .camera import CameraStream
from .qr_decoder import PreprocessedQRDecoder


class QRNode(Node):
    STABLE_DETECTIONS_REQUIRED = 5

    def __init__(self, **kwargs):
        super().__init__('qr_node', **kwargs)
        self.camera = None
        self.timer = None
        self.preview_created = False
        self.stable_text = None
        self.stable_count = 0
        self.result_published = False
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        defaults = {
            'camera_device': '/dev/video0',
            'camera_width': 640,
            'camera_height': 480,
            'camera_fps': 30.0,
            'camera_buffer_size': 4,
            'camera_dynamic_framerate': False,
            'scan_rate_hz': 30.0,
            'show_preview': False,
            'result_topic': '/qr/result',
            'center_topic': '/qr/center',
            'frame_id': 'qr_camera_optical_frame',
        }
        values = {}
        for name, default in defaults.items():
            values[name] = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)
            ).value
        for name in ('camera_width', 'camera_height', 'camera_fps', 'camera_buffer_size', 'scan_rate_hz'):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name in ('camera_device', 'result_topic', 'center_topic', 'frame_id'):
            if not values[name].strip():
                raise ValueError(f'{name} must not be empty')
        if values['result_topic'] == values['center_topic']:
            raise ValueError('result_topic and center_topic must be different')

        self.show_preview = values['show_preview']
        self.frame_id = values['frame_id']
        if self.show_preview:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise RuntimeError('show_preview=true needs a desktop display; use false for SSH')
            cv2.namedWindow('QR Scanner', cv2.WINDOW_NORMAL)
            self.preview_created = True

        # Volatile durability never replays an old detection to a late subscriber.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.result_publisher = self.create_publisher(String, values['result_topic'], qos)
        self.center_publisher = self.create_publisher(PointStamped, values['center_topic'], qos)
        self.decoder = PreprocessedQRDecoder()
        self.last_sequence = 0
        self.camera = CameraStream(
            values['camera_device'], values['camera_width'], values['camera_height'],
            values['camera_fps'], lambda: self.get_clock().now().to_msg(), self.get_logger(),
            buffer_size=values['camera_buffer_size'],
            dynamic_framerate=values['camera_dynamic_framerate'],
        )
        self.timer = self.create_timer(1.0 / values['scan_rate_hz'], self._scan)
        self.get_logger().info(
            f"Scanning started: {values['result_topic']} (String), "
            f"{values['center_topic']} (PointStamped); preview={self.show_preview}"
        )
        self.camera.start()

    def _scan(self):
        if self.result_published:
            return
        latest = self.camera.latest()
        if latest is None:
            self.stable_text = None
            self.stable_count = 0
            return
        sequence, frame, stamp, received_at = latest
        # A stalled device must not cause repeated publications of a frozen frame.
        if time.monotonic() - received_at > 0.5:
            self.stable_text = None
            self.stable_count = 0
            return
        if sequence == self.last_sequence:
            return
        self.last_sequence = sequence
        try:
            detection = self.decoder.decode(frame)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            self.stable_text = None
            self.stable_count = 0
            self.get_logger().warning(f'QR decode failed: {error}', throttle_duration_sec=5.0)
            return
        if detection is not None:
            if detection.text == self.stable_text:
                self.stable_count += 1
            else:
                self.stable_text = detection.text
                self.stable_count = 1
            if self.stable_count >= self.STABLE_DETECTIONS_REQUIRED:
                result = String(data=detection.text)
                center = PointStamped()
                center.header.stamp = stamp
                center.header.frame_id = self.frame_id
                center.point.x, center.point.y = detection.center
                center.point.z = 0.0
                self.result_publisher.publish(result)
                self.center_publisher.publish(center)
                self.result_published = True
                self.get_logger().info(
                    f'QR result stable after {self.stable_count} detections: {detection.text!r}; '
                    'closing camera'
                )
                self.timer.cancel()
                self.camera.close()
                if self.preview_created:
                    cv2.destroyWindow('QR Scanner')
                    self.preview_created = False
                return
        else:
            # A missing detection is a break in the consecutive-result window.
            self.stable_text = None
            self.stable_count = 0
        if self.show_preview:
            display = frame.copy()
            if detection is not None:
                cv2.polylines(display, [detection.polygon.astype(np.int32)], True, (0, 255, 0), 2)
                x, y = detection.center
                cv2.circle(display, (round(x), round(y)), 5, (0, 0, 255), -1)
                cv2.putText(
                    display, f'Center: ({x:.1f}, {y:.1f}) px', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
            cv2.imshow('QR Scanner', display)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self.preview_created:
            cv2.destroyWindow('QR Scanner')
            self.preview_created = False
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
