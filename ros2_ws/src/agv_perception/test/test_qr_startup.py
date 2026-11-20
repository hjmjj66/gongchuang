"""Bringup QR gating, with a fake camera and decoder and no hardware access."""

import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter

from agv_interfaces.msg import MissionState
from agv_perception.qr_node import QRNode


class QRStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_one_shot_result_waits_for_mission_and_subscriber(self):
        with patch('agv_perception.qr_node.CameraStream'), patch('agv_perception.qr_node.PreprocessedQRDecoder'):
            node = QRNode(parameter_overrides=[Parameter('wait_for_mission', value=True)])
            original = node.result_publisher
            try:
                node.timer.cancel()
                node.result_publisher = Mock()
                node.result_publisher.get_subscription_count.return_value = 1
                node._scan()
                node.camera.latest.assert_not_called()
                node._on_mission_state(MissionState(state='WAIT_SETUP'))
                node.result_publisher.get_subscription_count.return_value = 0
                node._scan()
                node.camera.latest.assert_not_called()
                node.result_publisher.get_subscription_count.return_value = 1
                node.decoder.decode.return_value = SimpleNamespace(text='156+123+516+231', center=(1., 2.))
                for seq in range(1, 6):
                    node.camera.latest.return_value = (seq, object(), node.get_clock().now().to_msg(), time.monotonic())
                    deadline = time.monotonic()+2.
                    while node.stable_count < seq and time.monotonic() < deadline:
                        node._scan()
                        time.sleep(.001)
                    self.assertEqual(node.stable_count, seq)
                node.result_publisher.publish.assert_called_once()
                node.camera.close.assert_called_once()
                node._scan()
                node.result_publisher.publish.assert_called_once()
            finally:
                node.result_publisher = original
                node.destroy_node()


if __name__ == '__main__':
    unittest.main()
