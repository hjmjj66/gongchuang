"""Exercise camera ownership without starting ROS or opening a device."""

from types import SimpleNamespace
import unittest

from competition_debug.node import CompetitionNode


class PreviewTests(unittest.TestCase):
    def test_manual_preview_reuses_vision_and_releases_before_competition(self):
        events = []
        vision = SimpleNamespace(open=lambda: events.append('open'),
                                 close=lambda: events.append('close'),
                                 preview=lambda: b'camera-jpeg')
        node = CompetitionNode.__new__(CompetitionNode)
        node.work = SimpleNamespace(vision=vision, released=lambda: True)
        node.lidar = None
        node.idle_preview = False
        node.idle_camera = ''
        node.idle_qr_camera = None
        node.session = SimpleNamespace(state='IDLE', radar=False)
        self.assertIsNone(node.preview())
        node.open_idle_preview()
        self.assertEqual(node.preview(), b'camera-jpeg')
        self.assertTrue(node.idle_preview)
        node.prepare_work()
        self.assertFalse(node.idle_preview)
        self.assertIsNone(node.preview())
        self.assertEqual(events, ['open', 'close'])

