"""Real process transport preserves detector output and failed-switch recovery."""

import unittest

import cv2
import numpy as np

from agv_perception.material_config import parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_worker import MaterialWorker


class MaterialWorkerTests(unittest.TestCase):
    def setUp(self):
        self.config = parse_material_config({'material_backend': 1})
        self.worker = MaterialWorker(self.config)
        self.addCleanup(self.worker.close)

    def image(self, width):
        hsv = np.full((200, width, 3), (0, 0, 255), np.uint8)
        cv2.circle(hsv, (80, 100), 25, (0, 200, 150), -1)
        cv2.circle(hsv, (width-80, 100), 25, (25, 170, 150), -1)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def test_shared_original_pixels_match_in_process_detection_and_resize(self):
        direct = MaterialDetector(self.config)
        for sequence, width in enumerate((300, 640, 300), 1):
            pixels = self.image(width)
            original = pixels.copy()
            result = self.worker.detect(pixels, sequence, 1.5, geometry_target=1)
            expected = direct.detect(pixels, sequence, 1.5, geometry_target=1)
            self.assertEqual(result.observations, expected.observations)
            self.assertEqual(result.image_size, (width, 200))
            self.assertEqual((result.frame_seq, result.stamp), (sequence, 1.5))
            self.assertEqual(result.status, expected.status)
            self.assertEqual(set(result.debug), {'contours'})
            self.assertGreaterEqual(result.stages_ms['handoff'], 0.)
            np.testing.assert_array_equal(pixels, original)

    def test_failed_model_switch_preserves_detector_and_next_frame(self):
        candidate = parse_material_config({'use_jhb_model': True,
                                           'jhb.model_path': '/tmp/missing_worker_model.onnx'})
        with self.assertRaisesRegex(RuntimeError, 'MODEL_LOAD_FAILED'):
            self.worker.configure(candidate)
        result = self.worker.detect(self.image(300), 2, 1.6, target_color_id=1)
        self.assertEqual({o.color_id for o in result.observations}, {1})
        self.assertTrue(all(o.valid for o in result.observations))

    def test_largest_per_color_is_forwarded_to_process(self):
        pixels = self.image(400)
        cv2.circle(pixels, (160, 100), 15, tuple(int(v) for v in pixels[100, 80]), -1)
        all_items = self.worker.detect(pixels).observations
        self.assertEqual([o.color_id for o in all_items], [1, 1, 2])
        result = self.worker.detect(pixels, largest_per_color=True)
        self.assertEqual([o.color_id for o in result.observations], [1, 2])
        self.assertAlmostEqual(result.observations[0].center_px[0], 80, delta=1)


if __name__ == '__main__':
    unittest.main()
