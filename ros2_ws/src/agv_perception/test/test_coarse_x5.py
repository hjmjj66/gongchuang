"""Opt-in production adapter with real models and replayed camera images."""

import logging
import os
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import cv2

from agv_perception.coarse_control import read_coarse_config
from agv_perception.coarse_vision import CoarseVision


class ReplayCamera:
    def __init__(self, *args):
        self.frame = None

    def start(self):
        pass

    def latest(self):
        return self.frame

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_TEST_X5_COARSE') == '1', 'requires X5 BPU and deployed models')
class JointModelTests(unittest.TestCase):
    def test_shared_runtime_both_real_models_without_camera(self):
        p = read_coarse_config(Path(__file__).parents[1]/'config/coarse.yaml')
        camera = ReplayCamera()
        with patch('agv_perception.work_vision.CameraStream', return_value=camera):
            worker = CoarseVision(p['vision'], logging.getLogger('joint_models'))
            try:
                cases = (
                    ('ring', '/root/ring_deploy_v1_20261103/images/check_254.jpg', {1, 2, 3}),
                    ('material', '/root/material_deploy_v1/export/upper_top/test_images/check.jpg', {1, 2, 4}),
                    ('material', '/root/material_deploy_v1/check_black_blues.jpg', {3, 5, 6}),
                )
                for sequence, (kind, path, expected) in enumerate(cases, 1):
                    worker.request(True, kind)
                    pixels = cv2.imread(path)
                    self.assertIsNotNone(pixels)
                    now = time.monotonic()
                    camera.frame = sequence, pixels, now, now
                    deadline = now+30.
                    obs = worker.latest()
                    while (obs is None or obs.sequence != sequence) and not worker.error and time.monotonic() < deadline:
                        time.sleep(.01)
                        obs = worker.latest()
                    self.assertFalse(worker.error, worker.error)
                    self.assertIsNotNone(obs)
                    self.assertEqual(obs.sequence, sequence)
                    actual = set(obs.rings) if kind == 'ring' else {c for c, _ in obs.materials}
                    self.assertEqual(actual, expected)
            finally:
                worker.close()
