"""Opt-in checks against the real deployed model and reference images on X5."""

import json
import os
from pathlib import Path
import unittest

import cv2
import numpy as np

from agv_perception.ring_detector import RingDetector
from agv_perception.ring_model import load_ring_model
from server import DEFAULT_CONFIG, read_config


@unittest.skipUnless(os.environ.get('AGV_TEST_X5_MODEL') == '1', 'set AGV_TEST_X5_MODEL=1 on X5')
class X5ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config, detector, geometry = read_config(DEFAULT_CONFIG)
        cls.deployment = Path(config['model_path']).parents[1]
        cls.model = load_ring_model(config['model_path'], config['model_format'],
                                    config['class_labels'], config['model_runtime_path'],
                                    config['confidence_threshold'], config['nms_threshold'])
        cls.detector = RingDetector(detector, geometry, cls.model)

    def test_model_boxes_match_previous_board_run_in_original_pixels(self):
        reference = json.loads((self.deployment / 'results/smoke_results.json').read_text())
        frame = cv2.imread(str(self.deployment / 'images/check_254.jpg'))
        original = frame.copy()
        boxes = self.model.infer(frame)
        expected = reference['tests'][0]['predictions']
        self.assertEqual({b.label for b in boxes}, {'1', '2', '3'})
        for box in boxes:
            item = next(p for p in expected if p['label'] == 'ring_' + box.label)
            np.testing.assert_allclose(box.bbox_xyxy, item['xyxy'], atol=0.1)
            self.assertAlmostEqual(box.confidence, item['confidence'], places=5)
        np.testing.assert_array_equal(frame, original)
        result = self.detector.detect_boxes(frame, boxes, 12, 3.5)
        self.assertEqual(result.image_size, (1280, 720))
        self.assertEqual(result.observations[1].center_source, 'circle')
        self.assertEqual(result.observations[2].center_source, 'circle')
        for item, expected_center in zip(result.observations[1:], ((367.5, 325.5), (661.5, 330.5))):
            np.testing.assert_allclose(item.circle_center_px, expected_center, atol=3)
        # The perspective-distorted first ring may fail geometry; it must not claim a circle.
        if result.observations[0].center_source == 'bbox':
            self.assertIsNone(result.observations[0].circle_center_px)

    def test_background_examples_below_threshold_do_not_produce_centers(self):
        for name in ('background_296.jpg', 'background_297.jpg'):
            with self.subTest(name=name):
                frame = cv2.imread(str(self.deployment / 'images' / name))
                result = self.detector.detect(frame)
                self.assertEqual(result.status, 'NO_DETECTION')
                self.assertTrue(all(o.center_px is None and o.bbox_xyxy is None for o in result.observations))


if __name__ == '__main__':
    unittest.main()
