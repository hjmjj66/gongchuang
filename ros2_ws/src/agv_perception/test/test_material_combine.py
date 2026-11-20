"""Explicit color detections exercise real matching, JHB inference and geometry."""

from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from agv_perception.material_combine import CombinedMaterialModel, match_colors
from agv_perception.material_config import JHB_DEPLOY, material_model_info, parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_jhb import JhbDetection
from agv_perception.material_model import MaterialModelDetection, MaterialModelError
from agv_perception.raw_control import RawTask
from agv_perception.work_config import read_work_config
from agv_perception.work_control import Observation


ONNX = Path(JHB_DEPLOY)/'detector/material_cap_v218_384x640_9a6d18d0cc69.onnx'


def color_box(color, box, score=.9):
    return MaterialModelDetection(color, f'color_{color}_top', score, box)


class CombineMatchingTests(unittest.TestCase):
    def setUp(self):
        self.p = parse_material_config({'test_combine': True})
        self.pairs = [(JhbDetection('cap_top', .95, (20,20,80,80)),
                       JhbDetection('material', .9, (10,10,90,130))),
                      (JhbDetection('cap_top', .95, (120,20,180,80)),
                       JhbDetection('material', .9, (110,10,190,130)))]

    def test_configuration_requires_both_models_and_overrides_hsv(self):
        p = parse_material_config({'test_combine': True, 'material_backend': 1,
                                   'hsv.red_ranges': 'unused', 'jhb.color_path': ''})
        self.assertEqual(material_model_info(p)['backend'], 'test_combine')
        self.assertEqual(material_model_info(p)['color_model_path'], p['top.model_path'])
        for change in ({'test_combine': 1}, {'use_jhb_model': True}, {'combine.min_iou': 0},
                       {'combine.min_iou': 1.1}, {'top.class_labels': []},
                       {'confidence_threshold': 'invalid'}, {'jhb.work_scale': 0}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_material_config({'test_combine': True, **change})

    def test_top_iou_matches_different_and_repeated_colors(self):
        for ids in ((2,3), (2,2)):
            colors = [color_box(ids[0], (23,23,83,83)), color_box(ids[1], (123,23,183,83))]
            matches = match_colors(self.pairs, colors[::-1], self.p)
            self.assertEqual(matches, [(colors[0], 'OK'), (colors[1], 'OK')])

    def test_whole_model_matches_material_box_and_requires_cap_inside(self):
        p = {**self.p, 'material_model': 2}
        color = color_box(2, (12,12,92,132))
        self.assertEqual(match_colors(self.pairs, [color], p),
                         [(color, 'OK'), (None, 'COLOR_UNMATCHED')])
        # Strong body overlap alone cannot label a cap outside the color box.
        self.assertEqual(match_colors(self.pairs[:1], [color_box(2, (10,55,90,130))], p),
                         [(None, 'COLOR_UNMATCHED')])

    def test_multiple_color_boxes_and_shared_color_box_are_ambiguous(self):
        colors = [color_box(2, (20,20,80,80)), color_box(3, (21,21,81,81))]
        self.assertEqual(match_colors(self.pairs[:1], colors, self.p),
                         [(None, 'COLOR_MATCH_AMBIGUOUS')])
        p = {**self.p, 'combine.min_iou': .2}
        shared = color_box(2, (20,20,180,80))
        self.assertEqual(match_colors(self.pairs, [shared], p),
                         [(None, 'COLOR_MATCH_AMBIGUOUS')]*2)

    def test_distant_low_score_and_large_enclosing_box_do_not_match(self):
        colors = [color_box(2, (20,20,80,80), .5),
                  color_box(3, (300,20,360,80)), color_box(4, (0,0,1000,1000))]
        self.assertEqual(match_colors(self.pairs, colors, self.p),
                         [(None, 'COLOR_UNMATCHED')]*2)


class ColorDetections:
    """Only the unavailable old X5 predictor is replaced with explicit inputs."""
    def __init__(self):
        self.boxes, self.image, self.error = [], None, None

    def infer(self, image):
        self.image = image
        if self.error:
            raise self.error
        return self.boxes


@unittest.skipUnless(ONNX.is_file(), 'JHB model package not installed')
class CombinedPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(2)

    def setUp(self):
        self.p = parse_material_config({'test_combine': True, 'jhb.model_path': str(ONNX),
                                       'jhb.color_path': '/missing/lab-is-not-used.json',
                                       'top.refine_circle': False})
        self.color = ColorDetections()
        with patch('agv_perception.material_combine.load_material_model', return_value=self.color):
            self.model = CombinedMaterialModel(self.p)
        self.detector = MaterialDetector(self.p, self.model)
        self.frame = cv2.imread(str(Path(JHB_DEPLOY)/'detector/selftest.jpg'))
        self.caps = [d for d in self.model.position.infer(self.frame) if d.label == 'cap_top']

    def test_same_frame_uses_old_color_and_jhb_center_without_loading_lab(self):
        for index, cap in enumerate(self.caps, 1):
            self.color.boxes.append(color_box(index, tuple(v+3 for v in cap.bbox_xyxy)))
        original = self.frame.copy()
        result = self.detector.detect(self.frame, 7, 1.5)
        self.assertEqual(result.backend, 'test_combine')
        self.assertEqual(result.status, 'OK')
        self.assertEqual(len(result.observations), 3)
        self.assertTrue(np.shares_memory(self.color.image, self.frame))
        self.assertFalse(hasattr(self.model.position, 'mu'))
        for item in result.observations:
            cap = self.caps[item.color_id-1]
            expected = ((cap.bbox_xyxy[0]+cap.bbox_xyxy[2])/2,
                        (cap.bbox_xyxy[1]+cap.bbox_xyxy[3])/2)
            self.assertEqual(item.center_px, expected)
            self.assertEqual(item.color_bbox_xyxy, self.color.boxes[item.color_id-1].bbox_xyxy)
            self.assertEqual(item.identity_source, 'yolo_jhb')
            self.assertEqual(item.color_confidence, .9)
            self.assertIsNone(item.color_d2)
        self.assertTrue({'jhb_yolo', 'color_yolo', 'matching', 'geometry'} <= result.stages_ms.keys())
        self.assertNotIn('color', result.stages_ms)
        np.testing.assert_array_equal(original, self.frame)

    def test_unmatched_center_reaches_initial_follow_but_cannot_grasp(self):
        result = self.detector.detect(self.frame, 1, .1)
        self.assertTrue(result.observations)
        self.assertTrue(all(o.color_id == 0 and not o.valid and o.center_px is not None
                            for o in result.observations))
        config = read_work_config(Path(__file__).resolve().parents[4]/'tools/coarse_debug/config/debug.yaml')['raw']
        task = RawTask(config, 'combined', (1,2,3), 0.)
        task.tick(.05, None, arm=SimpleNamespace(command_id=task.arm_request['command_id'], state='COMPLETED'))
        obs = Observation('material', 1, .1, result.image_size, {},
                          tuple((o.color_id, o.center_px) for o in result.observations))
        task.tick(.1, None, obs)
        self.assertEqual(task.state, 'INITIAL_FOLLOW')
        self.assertEqual(task.tracked_color, 0)
        self.assertTrue(task.servo_enabled)
        self.assertEqual(task.count, 0)
        self.assertIsNone(task.arm_request)

    def test_real_geometry_and_all_boxes_restore_roi_offsets(self):
        self.detector.config = {**self.p, 'top.refine_circle': True}
        frame = np.full((300,340,3), 240, np.uint8)
        cv2.circle(frame, (110,130), 50, (128,128,128), -1)
        boxes = [JhbDetection('cap_top', .95, (60,80,160,180)),
                 JhbDetection('material', .9, (50,70,170,220))]
        color = color_box(2, (64,84,164,184))
        roi = (20,30,360,330)
        result = self.detector._new_result(frame, 1, 0., roi)
        self.detector._combine_test(frame, boxes, [color], roi, 2, result, 2)
        item, = result.observations
        self.assertEqual((item.color_id, item.center_source, item.status), (2, 'circle', 'OK'))
        np.testing.assert_allclose(item.center_px, (130,160), atol=1)
        self.assertEqual(item.color_bbox_xyxy, (84,114,184,214))
        self.assertEqual(item.material_bbox_xyxy, (70,100,190,250))

    def test_color_failure_never_falls_back_to_lab_or_old_yolo_center(self):
        self.color.error = MaterialModelError('MODEL_INFERENCE_FAILED', 'color unavailable')
        result = self.detector.detect(self.frame)
        self.assertEqual(result.status, 'MODEL_INFERENCE_FAILED')
        self.assertEqual(result.observations, [])
        self.assertIn('color unavailable', result.error)


if __name__ == '__main__':
    unittest.main()
