import json
import unittest

import cv2
import numpy as np

from agv_perception.material_config import parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_model import MaterialModelDetection, MaterialModelError


def box(color=1, xy=(40,40,120,120), score=.9):
    return MaterialModelDetection(color,'red_top' if color == 1 else 'blue_top',score,xy)


class DetectorTests(unittest.TestCase):
    def test_target_color_refinement_keeps_other_colors_as_boxes(self):
        from unittest.mock import patch
        frame = np.full((220, 320, 3), 35, np.uint8)
        boxes = []
        for color, x in ((1, 60), (2, 160), (3, 260)):
            cv2.circle(frame, (x, 100), 30, (210, 210, 210), -1)
            boxes.append(box(color, (x-36, 64, x+36, 136)))
        detector = self.detector()
        with patch.object(detector.geometry, 'detect', wraps=detector.geometry.detect) as locate:
            result = detector.detect_boxes(frame, boxes, geometry_target=2)
            self.assertEqual(locate.call_count, 1)
        self.assertEqual([o.color_id for o in result.observations], [1, 2, 3])
        self.assertEqual([o.center_source for o in result.observations], ['bbox', 'circle', 'bbox'])
        self.assertEqual(result.observations[0].geometry_reason, 'NOT_TARGET')
        with patch.object(detector.geometry, 'detect', wraps=detector.geometry.detect) as locate:
            result = detector.detect_boxes(frame, boxes[:1], geometry_target=2)
            self.assertEqual(locate.call_count, 0)
            self.assertEqual(result.observations[0].center_source, 'bbox')
            detector.detect_boxes(frame, boxes)
            self.assertEqual(locate.call_count, 3)

    def setUp(self):
        self.frame = np.full((200,300,3),180,np.uint8)

    def detector(self, **params):
        return MaterialDetector(parse_material_config(params))

    def test_whole_has_no_geometry_and_keeps_multiple_same_color(self):
        detector = self.detector(material_model=2)
        detector.geometry = None
        result = detector.detect_boxes(self.frame,[box(),box(xy=(150,50,230,150))],12,3.5)
        self.assertEqual(len(result.observations),2)
        self.assertEqual(result.observations[1].center_px,(190,100))
        for o in result.observations:
            self.assertEqual((o.valid,o.status,o.center_source),(True,'BBOX_ONLY','bbox'))
            self.assertEqual(o.geometry_reason,'NOT_APPLICABLE')
            self.assertIsNone(o.circle_center_px)
        self.assertNotIn('geometry',result.stages_ms)

    def test_top_no_circle_falls_back_to_original_box(self):
        detector = self.detector(**{'top.roi_padding_ratio':.5})
        o = detector.detect_boxes(self.frame,[box()]).observations[0]
        self.assertEqual(o.roi_xyxy,(0,0,160,160))
        self.assertEqual(o.center_px,(80,80))
        self.assertIsNone(o.circle_center_px)
        self.assertEqual(o.geometry_reason,'NO_CIRCLE')

    def test_disable_fallback_has_no_valid_center(self):
        o = self.detector(**{'top.allow_bbox_fallback':False}).detect_boxes(self.frame,[box()]).observations[0]
        self.assertFalse(o.valid)
        self.assertIsNone(o.center_px)
        self.assertIsNone(o.center_source)
        self.assertEqual(o.bbox_center_px,(80,80))

    def test_disable_refine_ignores_fallback_switch(self):
        d = self.detector(**{'top.refine_circle':False,'top.allow_bbox_fallback':False})
        d.geometry = None
        o = d.detect_boxes(self.frame,[box()]).observations[0]
        self.assertEqual((o.center_source,o.geometry_reason),('bbox','DISABLED'))

    def test_strict_score_and_target_and_empty_results(self):
        d = self.detector(material_model=2,target_color_id=3)
        r = d.detect_boxes(self.frame,[box(),box(3,score=.5),box(3,score=.51)])
        self.assertEqual([o.color_id for o in r.observations],[3])
        self.assertEqual(d.detect_boxes(self.frame,[]).status,'NO_DETECTION')

    def test_largest_per_color_filters_score_before_area_for_all_six_colors(self):
        boxes = [candidate for color in range(1, 7) for candidate in (
            box(color, (10,10,290,190), .5),
            box(color, (20,20,280,180), .49),
            box(color, (30,30,60,60), .99),
            box(color, (100,50,180,120), .51))]

        class BoxInput:
            def infer(self, image):
                return boxes

        p = parse_material_config({'material_model': 2})
        detector = MaterialDetector(p, BoxInput())
        for detect in (lambda: detector.detect(self.frame, largest_per_color=True),
                       lambda: detector.detect_boxes(self.frame, boxes, largest_per_color=True)):
            result = detect()
            self.assertEqual([o.color_id for o in result.observations], list(range(1, 7)))
            self.assertTrue(all(o.bbox_xyxy == (100,50,180,120) for o in result.observations))
            self.assertTrue(all(o.yolo_confidence == .51 for o in result.observations))
        self.assertEqual(len(detector.detect(self.frame).observations), 12)

    def test_largest_box_without_center_does_not_fall_back_to_smaller_box(self):
        detector = self.detector(material_model=2)
        result = detector.detect_boxes(self.frame, [box(), box(xy=(310,20,500,190))],
                                       largest_per_color=True)
        self.assertEqual(len(result.observations), 1)
        self.assertIsNone(result.observations[0].center_px)
        self.assertEqual(result.status, 'NO_VALID_DETECTION')

    def test_confident_edge_boxes_are_usable_without_complete_visibility(self):
        boxes = [(0,30,80,100), (40,0,120,80), (240,30,300,100), (40,140,120,200),
                 (-10,30,80,100), (240,30,310,100)]
        for params, target in (({'material_model':2}, 0), ({}, 0),
                               ({'top.refine_circle':False}, 0), ({}, 1)):
            d = self.detector(**params)
            for xy in boxes:
                with self.subTest(params=params, target=target, xy=xy):
                    r = d.detect_boxes(self.frame, [box(6,xy)], geometry_target=target)
                    o = r.observations[0]
                    self.assertEqual((o.valid,o.status,o.center_source),(True,'BBOX_ONLY','bbox'))
                    self.assertEqual(o.center_px,((xy[0]+xy[2])/2,(xy[1]+xy[3])/2))
                    self.assertEqual(r.status,'OK')
        r = d.detect_boxes(self.frame,[box(6,(240,30,300,100),score=.5)])
        self.assertEqual(r.status,'NO_DETECTION')

    def test_boxes_fully_outside_image_have_no_valid_center(self):
        for xy in ((310,40,350,100), (-80,30,0,100), (40,-80,120,0), (40,200,120,280)):
            with self.subTest(xy=xy):
                r = self.detector().detect_boxes(self.frame,[box(xy=xy)])
                o = r.observations[0]
                self.assertEqual(o.status,'BOX_OUTSIDE_IMAGE')
                self.assertFalse(o.valid)
                self.assertIsNone(o.center_px)
                self.assertIsNone(o.center_source)
                self.assertEqual(r.status,'NO_VALID_DETECTION')

    def test_search_roi_edge_keeps_original_image_center(self):
        class BoxInput:
            def infer(self, image):
                return [box(6,(140,20,200,80))]
        d = MaterialDetector(parse_material_config({'search_roi':[50,40,250,180]}),BoxInput())
        o = d.detect(self.frame,target_color_id=6,geometry_target=6).observations[0]
        self.assertEqual((o.valid,o.center_source),(True,'bbox'))
        self.assertEqual(o.bbox_xyxy,(190,60,250,120))
        self.assertEqual(o.center_px,(220,90))

    def test_roi_offset_and_no_duplicate_letterbox(self):
        class BoxInput:
            def infer(self, image):
                self.shape = image.shape
                return [box(xy=(20,20,80,80))]
        model = BoxInput()
        d = MaterialDetector(parse_material_config({'material_model':2,'search_roi':[50,40,250,180]}),model)
        r = d.detect(self.frame,7,1.25)
        self.assertEqual(model.shape,(140,200,3))
        self.assertEqual(r.observations[0].bbox_xyxy,(70,60,130,120))
        self.assertEqual(r.observations[0].center_px,(100,90))
        direct = d.detect_boxes(self.frame,[box(xy=(20,20,80,80))])
        self.assertEqual(direct.observations[0].center_px,(50,50))

    def test_real_top_circle_and_manual_identity(self):
        frame = np.full((220,320,3),35,np.uint8)
        cv2.circle(frame,(160,100),40,(210,210,210),-1)
        original = frame.copy()
        r = self.detector().detect_roi(frame,(119,59,201,141),3,10,4.0)
        o = r.observations[0]
        self.assertTrue(o.valid)
        self.assertEqual(o.center_source,'circle')
        np.testing.assert_allclose(o.circle_center_px,(160,100),atol=2)
        self.assertEqual(o.identity_source,'manual')
        self.assertIsNone(o.yolo_confidence)
        json.dumps(r.to_dict(),allow_nan=False)
        np.testing.assert_array_equal(frame,original)

    def test_missing_model_and_failed_inference_not_silent_hsv(self):
        self.assertEqual(self.detector().detect(self.frame).status,'MODEL_NOT_CONFIGURED')
        class FailedInput:
            def infer(self, image):
                raise MaterialModelError('MODEL_INFERENCE_FAILED','test failure')
        d = MaterialDetector(parse_material_config({}),FailedInput())
        r = d.detect(self.frame)
        self.assertEqual(r.status,'MODEL_INFERENCE_FAILED')
        self.assertEqual(r.observations,[])

    def test_invalid_external_inputs(self):
        d = self.detector()
        for frame in (None,np.zeros((20,20),np.uint8),np.zeros((20,20,3),float)):
            with self.assertRaises(ValueError):
                d.detect(frame)
        with self.assertRaises(ValueError):
            d.detect(self.frame,search_roi=[0,0,500,200])
        with self.assertRaises(ValueError):
            d.detect_boxes(self.frame,[{'bbox':[]}])
        with self.assertRaises(ValueError):
            d.detect(self.frame,target_color_id=True)
        with self.assertRaises(ValueError):
            d.detect_roi(self.frame,(20,20,80,80),0)


if __name__ == '__main__':
    unittest.main()
