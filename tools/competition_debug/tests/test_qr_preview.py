"""Idle QR preview uses generated images and the real HTTP preview endpoint."""

import json
import os
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np

from test_ros_pty import SceneVision, SoftwareCamera

ROOT = Path(__file__).resolve().parents[3]


class PreviewCamera(SoftwareCamera):
    available = True
    stale = False

    def latest(self):
        if not self.available:
            return None
        latest = super().latest()
        if latest is not None and self.stale:
            return (*latest[:3], time.monotonic()-1.)
        return latest


@unittest.skipUnless(os.environ.get('AGV_COMPETITION_DEBUG_ROS_TEST') == '1',
                     'opt in to isolated software ROS/HTTP test')
class QRPreviewTests(unittest.TestCase):
    def test_idle_preview_switch_release_and_no_task_result(self):
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
        try:
            with patch('competition_debug.node.CameraStream', PreviewCamera):
                node = CompetitionNode(args)
                for timer in node.timers:
                    if timer is not node.idle_qr_timer:
                        timer.cancel()
                executor.add_node(node)
                node.work.vision = SceneVision(None)
                base = f'http://127.0.0.1:{node.web.http.server_port}'

                def command(action):
                    node.session.command(action, {}, 'qr-preview-test')
                    node._publish_web()

                def state():
                    with urlopen(base+'/api/state', timeout=2) as response:
                        return json.load(response)

                command('camera_open')
                self.assertIsNotNone(node.work.vision.camera)
                command('qr_camera_open')
                camera = node.idle_qr_camera
                self.assertIsNone(node.work.vision.camera)
                self.assertEqual(state()['camera_kind'], 'qr')
                deadline = time.monotonic()+3.
                while camera.sequence < 8 and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.05)
                self.assertGreaterEqual(camera.sequence, 8)
                self.assertFalse(camera.closed)
                self.assertEqual(node.session.state, 'IDLE')
                self.assertIsNone(node.session.qr_result)
                self.assertIsNone(node.qr)
                with urlopen(base+'/preview.jpg', timeout=2) as response:
                    self.assertEqual(response.headers['Content-Type'], 'image/jpeg')
                    pixels = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
                    self.assertEqual(pixels.shape, (480, 480, 3))

                camera.stale = True
                node.qr_preview_at -= 1.
                node._preview_idle_qr()
                node._publish_web()
                self.assertIn('断流', state()['camera'])
                with self.assertRaises(HTTPError) as error:
                    urlopen(base+'/preview.jpg', timeout=2)
                self.assertEqual(error.exception.code, 503)
                camera.available = False
                node.idle_qr_started_at -= 11.
                node._publish_web()
                self.assertIn('10秒', state()['camera'])

                command('camera_open')
                self.assertTrue(camera.closed)
                self.assertEqual(state()['camera_kind'], 'main')
                self.assertIsNotNone(node.work.vision.camera)
                for action in ('camera_close', 'stop', 'reset'):
                    command('qr_camera_open')
                    camera = node.idle_qr_camera
                    command(action)
                    self.assertTrue(camera.closed)
                    self.assertFalse(state()['camera_enabled'])
                command('qr_camera_open')
                camera = node.idle_qr_camera
                node.prepare_work()
                self.assertTrue(camera.closed)
                self.assertFalse(node.idle_preview)
                command('qr_camera_open')
                camera = node.idle_qr_camera
                node.close()
                self.assertTrue(camera.closed)
        finally:
            if node:
                node.release_camera()
                node.work.close()
                node.web.close()
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
