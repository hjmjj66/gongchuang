"""Competition QR decoder and ROS timer with generated QR frames, no camera."""

import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import yaml


@unittest.skipUnless(os.environ.get('AGV_ROUTE_DEBUG_ROS_TEST') == '1', 'opt in to isolated ROS test')
class QRScanTests(unittest.TestCase):
    def test_real_decoder_five_frames_release_restart_and_invalid_code(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from route_debug.qr_scan import QRScan

        class FrameCamera:
            def __init__(self, device, width, height, fps, timestamp, logger, **kwargs):
                self.timestamp = timestamp
                self.closed = False
                self.sequence = 0
                self.frame = None
                self.stale = False

            def start(self):
                pass

            def latest(self):
                if self.frame is None or self.closed:
                    return None
                self.sequence += 1
                return (self.sequence, self.frame, self.timestamp(),
                        time.monotonic()-(1. if self.stale else 0.))

            def close(self):
                self.closed = True

        def image(text):
            qr = cv2.QRCodeEncoder_create().encode(text)
            qr = cv2.copyMakeBorder(qr, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=255)
            return cv2.cvtColor(cv2.resize(qr, (480, 480), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)

        rclpy.init()
        owner = rclpy.create_node('qr_debug_test_owner')
        executor = SingleThreadedExecutor()
        executor.add_node(owner)
        scanner = None
        try:
            with tempfile.TemporaryDirectory() as folder, patch('agv_perception.qr_node.CameraStream', FrameCamera):
                path = Path(folder)/'qr.yaml'
                path.write_text(yaml.safe_dump({'qr_node': {'ros__parameters': {
                    'wait_for_mission': True, 'show_preview': True, 'camera_device': '/no-real-device'}}}))
                for text in ('156+123+516+231', '156+123+516+231', 'invalid-task'):
                    scanner = QRScan(owner, path)
                    camera = scanner.node.camera
                    self.assertEqual(camera.sequence, 0)
                    self.assertFalse(scanner.node.wait_for_mission)
                    self.assertFalse(scanner.node.show_preview)
                    self.assertIsNone(scanner.tick())
                    camera.frame = image(text)
                    for expected_count in range(1, 5):
                        until = time.monotonic()+3.
                        while scanner.node.stable_count < expected_count and time.monotonic() < until:
                            executor.spin_once(timeout_sec=.05)
                        self.assertEqual(scanner.node.stable_count, expected_count)
                        self.assertIsNone(scanner.tick())
                    until = time.monotonic()+3.
                    while not scanner.node.result_published and time.monotonic() < until:
                        executor.spin_once(timeout_sec=.05)
                    self.assertTrue(scanner.node.result_published)
                    self.assertTrue(camera.closed)
                    if text == 'invalid-task':
                        with self.assertRaises(ValueError):
                            scanner.tick()
                    else:
                        self.assertEqual(scanner.tick(), text)
                    scanner.close()
                    scanner = None
                    self.assertEqual(executor.get_nodes(), [owner])
                scanner = QRScan(owner, path)
                scanner.started_at -= 11.
                with self.assertRaisesRegex(RuntimeError, '10秒'):
                    scanner.tick()
                scanner.node.camera.frame = image('156+123+516+231')
                scanner.node.camera.stale = True
                with self.assertRaisesRegex(RuntimeError, '断流'):
                    scanner.tick()
                camera = scanner.node.camera
                executor.remove_node(owner)
                scanner.close()
                scanner = None
                self.assertTrue(camera.closed)
                executor.add_node(owner)
        finally:
            if scanner:
                scanner.close()
            executor.remove_node(owner)
            owner.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
