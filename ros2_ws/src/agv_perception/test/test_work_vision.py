"""Production adapter exercises the same camera/process/JPEG worker as 8086."""

import logging
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from agv_perception.coarse_vision import CoarseVision
from agv_perception.work_vision import RingVision


class ReplayCamera:
    def __init__(self, *args):
        self.frame = None
        self.closed = False

    def start(self):
        pass

    def latest(self):
        return self.frame

    def close(self):
        self.closed = True

    def push(self, sequence):
        pixels = np.full((200, 400, 3), (0, 0, 255), np.uint8)
        for x, radius, hue in ((60, 25, 0), (150, 15, 0), (240, 15, 25), (330, 25, 25)):
            cv2.circle(pixels, (x, 100), radius, (hue, 200, 150), -1)
        now = time.monotonic()
        self.frame = sequence, cv2.cvtColor(pixels, cv2.COLOR_HSV2BGR), now, now


class EmptyRingModel:
    def infer(self, pixels):
        return []


class WorkVisionTests(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            self.assertFalse(self.worker.error, self.worker.error)
            if predicate():
                return
            time.sleep(.005)
        self.fail('shared vision worker did not publish a result')

    def setUp(self):
        config = Path(__file__).parents[1]/'config'
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        material = yaml.safe_load((config/'material.yaml').read_text())
        material['material_debug_node']['ros__parameters'].update(material_backend=1, use_jhb_model=False)
        path = Path(folder.name)/'material.yaml'
        path.write_text(yaml.safe_dump(material))
        self.camera = ReplayCamera()
        camera_patch = patch('agv_perception.work_vision.CameraStream', return_value=self.camera)
        model_patch = patch('agv_perception.work_vision.load_ring_model', return_value=EmptyRingModel())
        camera_patch.start(); model_patch.start()
        self.addCleanup(camera_patch.stop); self.addCleanup(model_patch.stop)
        p = dict(ring_config=str(config/'ring.yaml'), material_config=str(path))
        self.worker = CoarseVision(p, logging.getLogger('production_vision'), raw_config=p)
        self.addCleanup(self.worker.close)
        self.wait_for(lambda: self.worker.ready)
        self.assertIsInstance(self.worker.camera, RingVision)

    def test_raw_largest_selection_and_storage_instances_use_same_live_camera(self):
        for sequence, area, expected in ((1, 'raw', [1, 2]), (2, 'coarse', [1, 1, 2, 2])):
            self.worker.request(True, 'material', area=area)
            self.camera.push(sequence)
            self.wait_for(lambda: self.worker.latest() is not None and self.worker.latest().sequence == sequence)
            self.assertEqual([c for c, _ in self.worker.latest().materials], expected)
            self.assertFalse(self.camera.closed)

    def test_action_pause_and_target_switch_discard_previous_frames(self):
        self.worker.request(True, 'material', area='raw', geometry_target=1)
        self.camera.push(1)
        self.wait_for(lambda: self.worker.latest() is not None)
        self.worker.request(True, area='raw')
        self.assertIsNone(self.worker.latest())
        self.camera.push(2)
        self.worker.request(True, 'material', area='raw', geometry_target=2)
        time.sleep(.04)
        self.assertIsNone(self.worker.latest())
        self.camera.push(3)
        self.wait_for(lambda: self.worker.latest() is not None)
        obs = self.worker.latest()
        self.assertEqual((obs.sequence, obs.geometry_target), (3, 2))
        self.assertEqual({c for c, _ in obs.materials}, {1, 2})
        self.worker.request(False)
        self.assertIsNone(self.worker.latest())
        self.worker.close()
        self.assertTrue(self.camera.closed)
