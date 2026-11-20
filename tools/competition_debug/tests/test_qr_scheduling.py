"""QR decoding must not block the controller's 200 ms arm feedback check."""

import os
from pathlib import Path
import threading
import time
from unittest.mock import patch
import unittest

from test_ros_pty import SoftwareCamera


@unittest.skipUnless(os.environ.get('AGV_COMPETITION_DEBUG_ROS_TEST') == '1',
                     'opt in to isolated software ROS test')
class QRSchedulingTests(unittest.TestCase):
    def test_slow_start_decode_cleanup_and_stop_during_start_keep_feedback_running(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from agv_interfaces.msg import ArmExecution
        from agv_perception.qr_node import QRNode
        from agv_perception.qr_decoder import PreprocessedQRDecoder
        from agv_mission.task_code import parse_task_code
        from route_debug.qr_scan import QRScan

        startup_entered, startup_release = threading.Event(), threading.Event()
        decode_entered, decode_release = threading.Event(), threading.Event()
        close_entered, close_release = threading.Event(), threading.Event()
        cameras = []

        class SlowNode(QRNode):
            def __init__(self, **kwargs):
                startup_entered.set()
                startup_release.wait(3.)
                super().__init__(**kwargs)

        class SlowDecoder(PreprocessedQRDecoder):
            def decode(self, frame):
                decode_entered.set()
                decode_release.wait(3.)
                return super().decode(frame)

        class SlowCamera(SoftwareCamera):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                cameras.append(self)

            def close(self):
                close_entered.set()
                close_release.wait(3.)
                super().close()

        rclpy.init()
        executor = SingleThreadedExecutor()
        owner = rclpy.create_node('qr_lifecycle_test')
        executor.add_node(owner)
        publisher = owner.create_publisher(ArmExecution, '/test/qr_arm', 1)
        received = []
        owner.create_timer(.02, lambda: publisher.publish(ArmExecution(state='IDLE', idle=True)))
        owner.create_subscription(ArmExecution, '/test/qr_arm',
                                  lambda msg: received.append(time.monotonic()), 1)
        scanner = None
        cleanup = None

        def spin_for(duration):
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)

        def feedback_continues(duration):
            before = len(received)
            spin_for(duration)
            self.assertGreater(len(received), before+2)
            self.assertLess(max(b-a for a, b in zip(received, received[1:])), .2)

        def tick():
            if scanner is not None:
                scanner.tick()

        owner.create_timer(.02, tick)
        config = Path(__file__).resolve().parents[3]/'ros2_ws/src/agv_perception/config/qr.yaml'
        try:
            spin_for(.2)
            with patch('route_debug.qr_scan.QRNode', SlowNode), \
                    patch('agv_perception.qr_node.CameraStream', SlowCamera), \
                    patch('agv_perception.qr_node.PreprocessedQRDecoder', SlowDecoder):
                started = time.monotonic()
                scanner = QRScan(owner, config, result_validator=lambda text: bool(parse_task_code(text)),
                                 stable_detections_required=1, wait_for_camera=True,
                                 background_lifecycle=True, accept_delayed_result=True)
                self.assertLess(time.monotonic()-started, .1)
                self.assertTrue(startup_entered.wait(1.))
                feedback_continues(.3)
                startup_release.set()
                deadline = time.monotonic()+3.
                while not decode_entered.is_set() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(decode_entered.is_set())
                feedback_continues(.6)
                decode_release.set()
                deadline = time.monotonic()+3.
                while not scanner.node.result_published and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertEqual(scanner.tick(), SoftwareCamera.text)
                self.assertEqual(scanner.node.stable_count, 1)
                self.assertFalse(close_entered.is_set())
                cleanup = scanner.close()
                scanner = None
                self.assertTrue(close_entered.wait(1.))
                feedback_continues(.3)
                close_release.set()
                cleanup.result(timeout=3.)
                self.assertTrue(cameras[-1].closed)
                self.assertEqual(executor.get_nodes(), [owner])

                startup_entered.clear()
                startup_release.clear()
                scanner = QRScan(owner, config, background_lifecycle=True)
                self.assertTrue(startup_entered.wait(1.))
                cleanup = scanner.close()
                scanner = None
                feedback_continues(.3)
                startup_release.set()
                cleanup.result(timeout=3.)
                self.assertTrue(cameras[-1].closed)
                self.assertEqual(executor.get_nodes(), [owner])
        finally:
            startup_release.set()
            decode_release.set()
            close_release.set()
            if scanner is not None:
                cleanup = scanner.close()
            if cleanup is not None:
                cleanup.result(timeout=5.)
            executor.remove_node(owner)
            owner.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

    def test_feedback_keeps_running_during_slow_decode(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from agv_interfaces.msg import ArmExecution
        from agv_perception.qr_decoder import PreprocessedQRDecoder
        from agv_perception.qr_node import QRNode

        rclpy.init()
        executor = SingleThreadedExecutor()
        owner = rclpy.create_node('qr_feedback_test')
        publisher = owner.create_publisher(ArmExecution, '/test/qr_arm', 1)
        received = []
        checks = []
        started = threading.Event()
        release = threading.Event()

        def publish():
            message = ArmExecution(state='IDLE', idle=True)
            message.header.stamp = owner.get_clock().now().to_msg()
            publisher.publish(message)

        owner.create_timer(.02, publish)
        owner.create_subscription(ArmExecution, '/test/qr_arm',
                                  lambda message: received.append(time.monotonic()), 1)
        owner.create_timer(.02, lambda: checks.append(time.monotonic()))

        class DelayedDecoder(PreprocessedQRDecoder):
            def decode(self, frame):
                started.set()
                release.wait(.3)
                return super().decode(frame)

        qr = None
        try:
            executor.add_node(owner)
            deadline = time.monotonic()+2.
            while len(received) < 3 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            self.assertGreaterEqual(len(received), 3)
            with patch('agv_perception.qr_node.CameraStream', SoftwareCamera), \
                    patch('agv_perception.qr_node.PreprocessedQRDecoder', DelayedDecoder):
                qr = QRNode()
                executor.add_node(qr)
                deadline = time.monotonic()+3.
                while not started.is_set() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(started.is_set())
                received_before = len(received)
                deadline = time.monotonic()+.15
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertGreater(len(received), received_before)
                self.assertLess(max(b-a for a, b in zip(checks, checks[1:])), .2)
                release.set()
                deadline = time.monotonic()+5.
                while not qr.result_published and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(qr.result_published)
                self.assertEqual(qr.stable_text, SoftwareCamera.text)
                self.assertTrue(qr.camera.closed)
        finally:
            release.set()
            if qr:
                executor.remove_node(qr)
                qr.destroy_node()
            executor.remove_node(owner)
            owner.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
