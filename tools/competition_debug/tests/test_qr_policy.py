"""Competition waits through camera gaps and accepts one valid captured frame."""

import os
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_ros_pty import SoftwareCamera

ROOT = Path(__file__).resolve().parents[3]


class FrameCamera(SoftwareCamera):
    frame = None

    def latest(self):
        return None if self.closed else self.frame

    def capture(self, text, age=0.):
        self.text = text
        sequence, pixels, stamp, received_at = super().latest()
        self.frame = sequence, pixels, stamp, received_at-age


@unittest.skipUnless(os.environ.get('AGV_COMPETITION_DEBUG_ROS_TEST') == '1',
                     'opt in to isolated software ROS test')
class QRPolicyTests(unittest.TestCase):
    def test_wait_for_camera_and_first_valid_frame(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from competition_debug.node import CompetitionNode

        args = SimpleNamespace(host='127.0.0.1', port=0, screen_port=None,
            debug_config=str(ROOT/'tools/route_debug/config/debug.yaml'))
        for name, package in (('planner', 'agv_planning'), ('mission', 'agv_mission'),
                              ('cboard', 'agv_cboard'), ('screen', 'agv_screen'),
                              ('qr', 'agv_perception'), ('raw', 'agv_perception'),
                              ('coarse', 'agv_perception'), ('storage', 'agv_perception')):
            setattr(args, name+'_config', str(ROOT/'ros2_ws/src'/package/'config'/(name+'.yaml')))
        args.work_config = args.coarse_config
        rclpy.init()
        executor = SingleThreadedExecutor()
        node = None
        master, slave = os.openpty()
        os.set_blocking(master, False)
        args.screen_port = os.ttyname(slave)
        try:
            with patch('agv_perception.qr_node.CameraStream', FrameCamera):
                node = CompetitionNode(args)
                for timer in node.timers:
                    timer.cancel()
                executor.add_node(node)
                node.start_qr()
                deadline = time.monotonic()+3.
                while node.qr.node is None and time.monotonic() < deadline:
                    node.tick_qr()
                    executor.spin_once(timeout_sec=.01)
                qr = node.qr.node
                camera = qr.camera
                self.assertEqual(qr.stable_detections_required, 1)
                node.qr.started_at -= 3600.
                self.assertIsNone(node.tick_qr())
                qr._scan()
                self.assertIsNone(qr.decode_future)

                camera.capture(SoftwareCamera.text, age=1.)
                self.assertIsNone(node.tick_qr())
                qr._scan()
                self.assertIsNone(qr.decode_future)
                self.assertFalse(qr.result_published)

                for text in ('invalid-task', '111+123+111+123'):
                    camera.capture(text)
                    qr._scan()
                    qr.decode_future.result(timeout=3.)
                    qr._scan()
                    self.assertFalse(qr.result_published)
                    self.assertIsNone(node.tick_qr())
                    self.assertIn('任务码无效', node.qr_wait)
                    self.assertEqual(qr.stable_count, 0)
                log_count = len(node.session.log)
                qr._scan()
                self.assertIsNone(qr.decode_future)
                self.assertEqual(len(node.session.log), log_count)

                camera.frame = None
                self.assertIsNone(node.tick_qr())
                qr._scan()
                camera.capture('354+321+543+213')
                sequence = camera.sequence
                deadline = time.monotonic()+3.
                while not qr.result_published and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                result = node.tick_qr()
                self.assertEqual(result, '354+321+543+213')
                self.assertEqual(qr.stable_count, 1)
                self.assertEqual(qr.last_sequence, sequence)
                self.assertEqual(camera.sequence, sequence)
                node.stop_qr()
                deadline = time.monotonic()+3.
                while not camera.closed and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(camera.closed)
                node.screen.start()
                node.screen.stop()
                node.show_qr(result)
                output = b''
                for _ in range(2):
                    node.screen.next_send = 0.
                    node.screen.tick(lambda *args: self.fail('screen setup is closed'))
                    output += os.read(master, 4096)
                self.assertEqual(output, b'show_number.t0.txt="354+321"\xff\xff\xff'
                                         b'show_number.t1.txt="543+213"\xff\xff\xff')
                self.assertEqual(executor.get_nodes(), [node])
                self.assertEqual(node.session.state, 'IDLE')
                self.assertIsNone(node.session.qr_result)
        finally:
            if node:
                node.stop_qr()
                for future in node.qr_cleanup:
                    future.result(timeout=5.)
                node.screen.close()
                node.release_camera()
                node.work.close()
                node.web.close()
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)
