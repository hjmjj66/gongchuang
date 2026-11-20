"""Frame/box contracts and real geometry without any fabricated model runtime."""

from dataclasses import fields, replace
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
import yaml

from agv_perception.ring_detector import DetectorConfig, RingDetector, select_best_boxes
from agv_perception.ring_geometry import GeometryConfig
from agv_perception.ring_model import ModelDetection, RingModelError, load_ring_model


class DetectorTests(unittest.TestCase):
    def test_only_target_runs_geometry_in_serial_and_parallel_paths(self):
        from concurrent.futures import ThreadPoolExecutor
        from unittest.mock import patch
        frame = np.full((320, 600, 3), 240, np.uint8)
        boxes = []
        for ring, x in ((1, 100), (2, 300), (3, 500)):
            cv2.circle(frame, (x, 160), 55, (0, 0, 0), 3, cv2.LINE_AA)
            boxes.append(ModelDetection(str(ring), .9, (x-70, 90, x+70, 230)))
        with ThreadPoolExecutor(max_workers=3) as pool:
            for executor in (None, pool):
                detector = RingDetector(roi_executor=executor)
                with patch.object(detector.geometry, 'detect', wraps=detector.geometry.detect) as locate:
                    result = detector.detect_boxes(frame, boxes, geometry_target=2)
                    self.assertEqual(locate.call_count, 1)
                for item in result.observations:
                    self.assertTrue(item.valid)
                    self.assertEqual(item.center_source, 'circle' if item.ring_id == 2 else 'bbox')
                    if item.ring_id != 2:
                        self.assertEqual(item.geometry_reason, 'NOT_TARGET')
                        self.assertIsNone(item.circle_center_px)
                self.assertEqual(set(result.debug), {2})
                with patch.object(detector.geometry, 'detect', wraps=detector.geometry.detect) as locate:
                    detector.detect_boxes(frame, boxes[:1], geometry_target=2)
                    self.assertEqual(locate.call_count, 0)
                    detector.detect_boxes(frame, boxes)
                    self.assertEqual(locate.call_count, 3)
        for invalid in (-1, 4, True, None):
            with self.assertRaises(ValueError):
                RingDetector().detect_boxes(frame, boxes, geometry_target=invalid)

    def setUp(self):
        self.frame = np.full((400, 600, 3), 240, np.uint8)
        self.detector = RingDetector()

    def test_highest_per_id_and_strict_threshold(self):
        boxes = [
            ModelDetection('1', 0.6, (20, 20, 100, 100)),
            ModelDetection('1', 0.9, (30, 30, 110, 110)),
            ModelDetection('2', 0.750001, (120, 20, 200, 100)),
            ModelDetection('3', 0.75, (220, 20, 300, 100)),
            ModelDetection('other', 0.99, (320, 20, 400, 100)),
        ]
        selected = select_best_boxes(boxes, 0.75)
        self.assertEqual(set(selected), {1, 2})
        self.assertEqual(selected[1], boxes[1])
        result = self.detector.detect_boxes(self.frame, boxes, 7, 12.5)
        self.assertEqual([o.ring_id for o in result.observations], [1, 2, 3])
        self.assertEqual(result.observations[0].center_px, (70, 70))
        self.assertFalse(result.observations[2].valid)
        self.assertIsNone(result.observations[2].center_px)
        self.assertEqual(result.status, 'PARTIAL')
        self.assertEqual(result.frame_seq, 7)
        self.assertEqual(result.stamp, 12.5)

    def test_threshold_applies_independently_to_all_three_ids(self):
        boxes = [ModelDetection(str(i), confidence, (i * 100, 20, i * 100 + 80, 100))
                 for i, confidence in ((1, 0.749999), (2, 0.75), (3, 0.750001))]
        self.assertEqual(set(select_best_boxes(boxes, 0.75)), {3})

    def test_equal_confidence_keeps_first(self):
        first = ModelDetection('1', 0.9, (20, 20, 100, 100))
        second = ModelDetection('1', 0.9, (120, 20, 200, 100))
        self.assertEqual(select_best_boxes([first, second], 0.5)[1], first)

    def test_numpy_model_coordinates_are_normalized_for_json(self):
        box = ModelDetection('1', np.float32(0.8),
                             tuple(np.float32(v) for v in (20, 20, 100, 100)))
        result = self.detector.detect_boxes(self.frame, [box])
        encoded = json.dumps(result.to_dict(), allow_nan=False)
        self.assertEqual(json.loads(encoded)['observations'][0]['center_px'], [60.0, 60.0])

    def test_lower_confidence_geometry_never_replaces_selected_box(self):
        cv2.circle(self.frame, (300, 200), 60, (0, 0, 0), 3, cv2.LINE_AA)
        boxes = [ModelDetection('1', 0.9, (30, 30, 150, 150)),
                 ModelDetection('1', 0.8, (230, 130, 370, 270))]
        result = self.detector.detect_boxes(self.frame, boxes)
        self.assertEqual(result.observations[0].center_source, 'bbox')
        self.assertEqual(result.observations[0].center_px, (90, 90))

    def test_real_circle_restored_to_original_coordinates_without_mutation(self):
        cv2.circle(self.frame, (250, 180), 75, (0, 0, 0), 3, cv2.LINE_AA)
        original = self.frame.copy()
        result = self.detector.detect_boxes(
            self.frame, [ModelDetection('2', 0.8, (170, 100, 330, 260))],
        )
        observation = result.observations[1]
        self.assertEqual(observation.center_source, 'circle')
        self.assertAlmostEqual(observation.center_px[0], 250, delta=3)
        self.assertAlmostEqual(observation.center_px[1], 180, delta=3)
        self.assertEqual(observation.identity_source, 'yolo')
        self.assertEqual(observation.bbox_center_px, (250, 180))
        self.assertEqual(observation.circle_center_px, observation.center_px)
        np.testing.assert_array_equal(self.frame, original)
        self.assertNotIn('debug', result.to_dict())
        json.dumps(result.to_dict(), allow_nan=False)

    def test_expanded_roi_clipping_does_not_shift_bbox_fallback(self):
        detector = RingDetector(replace(DetectorConfig(), roi_padding_ratio=0.5))
        result = detector.detect_roi(self.frame, (10, 10, 110, 110), 1)
        observation = result.observations[0]
        self.assertEqual(observation.roi_xyxy, (0, 0, 160, 160))
        self.assertEqual(observation.center_px, (60, 60))
        self.assertEqual(observation.status, 'BBOX_ONLY')
        self.assertEqual(observation.identity_source, 'manual')
        self.assertIsNone(observation.yolo_confidence)

    def test_parallel_rois_match_serial_geometry_and_keep_id_order(self):
        boxes = []
        for ring_id, x in ((3, 480), (1, 100), (2, 290)):
            cv2.circle(self.frame, (x, 200), 60, (0, 0, 0), 3, cv2.LINE_AA)
            boxes.append(ModelDetection(str(ring_id), 0.9, (x - 80, 120, x + 80, 280)))
        serial = self.detector.detect_boxes(self.frame, boxes)
        with ThreadPoolExecutor(max_workers=3) as pool:
            parallel = RingDetector(roi_executor=pool).detect_boxes(self.frame, boxes)
        self.assertEqual(parallel.observations, serial.observations)
        self.assertTrue(all(o.center_source == 'circle' for o in parallel.observations))
        self.assertEqual([o.ring_id for o in parallel.observations], [1, 2, 3])

    def test_original_frame_edge_disables_fallback(self):
        result = self.detector.detect_roi(self.frame, (0, 20, 100, 120))
        observation = result.observations[0]
        self.assertEqual(observation.status, 'TRUNCATED')
        self.assertFalse(observation.valid)
        self.assertIsNone(observation.center_px)

    def test_trusted_circle_still_allowed_when_original_box_touches_edge(self):
        cv2.circle(self.frame, (80, 100), 72, (0, 0, 0), 3, cv2.LINE_AA)
        result = self.detector.detect_roi(self.frame, (0, 20, 160, 180), 1)
        self.assertEqual(result.observations[0].center_source, 'circle')

    def test_fallback_can_be_disabled_and_off_image_box_is_invalid(self):
        detector = RingDetector(replace(DetectorConfig(), allow_bbox_fallback=False))
        observation = detector.detect_roi(self.frame, (20, 20, 100, 100)).observations[0]
        self.assertFalse(observation.valid)
        self.assertEqual(observation.status, 'GEOMETRY_FAILED')
        observation = detector.detect_roi(self.frame, (800, 20, 900, 100)).observations[0]
        self.assertEqual(observation.status, 'BOX_OUTSIDE_IMAGE')

    def test_small_roi_has_explicit_geometry_reason(self):
        observation = self.detector.detect_roi(self.frame, (20, 20, 25, 25)).observations[0]
        self.assertEqual(observation.geometry_reason, 'ROI_TOO_SMALL')
        self.assertEqual(observation.center_source, 'bbox')
        self.assertIsNone(observation.circle_center_px)
        self.assertEqual(observation.bbox_center_px, observation.center_px)

    def test_no_boxes_cannot_reuse_previous_coordinates(self):
        self.detector.detect_boxes(self.frame, [ModelDetection('1', 0.8, (20, 20, 100, 100))])
        result = self.detector.detect_boxes(self.frame, [], 2, 3.0)
        self.assertEqual(result.status, 'NO_DETECTION')
        self.assertTrue(all(not o.valid and o.center_px is None for o in result.observations))

    def test_invalid_external_image_or_box_is_rejected(self):
        with self.assertRaises(ValueError):
            self.detector.detect_roi(np.zeros((5, 5), np.uint8), (0, 0, 4, 4))
        for box in ((10, 10, 0, 0), (0, 0, float('nan'), 10)):
            with self.subTest(box=box), self.assertRaises(ValueError):
                self.detector.detect_roi(self.frame, box)
        with self.assertRaises(ValueError):
            select_best_boxes([ModelDetection('1', float('nan'), (0, 0, 20, 20))], 0.5)

    def test_missing_model_is_not_a_geometry_fallback(self):
        with self.assertRaises(RingModelError) as context:
            self.detector.detect(self.frame)
        self.assertEqual(context.exception.status, 'MODEL_NOT_CONFIGURED')
        with self.assertRaises(RingModelError) as context:
            load_ring_model()
        self.assertEqual(context.exception.status, 'MODEL_NOT_CONFIGURED')

    def test_nonempty_model_still_requires_real_adapter(self):
        with self.assertRaises(RingModelError) as context:
            load_ring_model('/nonexistent/agv_ring_model', 'unconfigured')
        self.assertEqual(context.exception.status, 'MODEL_LOAD_FAILED')
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / 'placeholder'
            model_path.write_bytes(b'')
            with self.assertRaises(RingModelError) as context:
                load_ring_model(str(model_path), 'future-format')
            self.assertEqual(context.exception.status, 'MODEL_ADAPTER_NOT_IMPLEMENTED')

    def test_yaml_defaults_match_the_actual_config_classes(self):
        config_path = Path(__file__).resolve().parents[1] / 'config' / 'ring.yaml'
        config = yaml.safe_load(config_path.read_text())['ring_debug_node']['ros__parameters']
        self.assertEqual(config['model_format'], 'rdk_yolov8_bin')
        self.assertTrue(config['model_path'].endswith('.bin'))
        self.assertTrue(config['model_runtime_path'].endswith('rdk_model_zoo'))
        for cls, prefix in ((DetectorConfig, ''), (GeometryConfig, 'geometry.')):
            for item in fields(cls):
                self.assertEqual(config[prefix + item.name], getattr(cls(), item.name))


if __name__ == '__main__':
    unittest.main()
