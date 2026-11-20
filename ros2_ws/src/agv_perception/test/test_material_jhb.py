"""Numeric contracts, real geometry and optional packaged-model regression."""

import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from agv_perception.material_config import JHB_DEPLOY, material_model_info, parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_jhb import JhbDetection, class_nms, decode
from agv_perception.material_model import MaterialModelError, load_material_model
from agv_perception.ring_geometry import GeometryConfig, RingGeometry

ROOT = Path(JHB_DEPLOY)
ONNX = ROOT / 'detector/material_cap_v218_384x640_9a6d18d0cc69.onnx'


class JhbContractTests(unittest.TestCase):
    def test_mode_overrides_old_backend_and_reports_actual_model(self):
        p = parse_material_config({'use_jhb_model': True, 'material_backend': 1,
                                   'material_model': 2, 'whole.model_path': '',
                                   'hsv.red_ranges': 'unused'})
        self.assertEqual(material_model_info(p)['backend'], 'jhb')
        self.assertEqual(material_model_info(p)['class_labels'], ['material','cap_top'])
        self.assertEqual(material_model_info(p)['model_path'], p['jhb.model_path'])

    def test_invalid_parameters_and_missing_model(self):
        for change in ({'use_jhb_model':1}, {'jhb.work_scale':0},
                       {'jhb.model_path':'wrong.pt'}, {'jhb.color_p_min':1.1},
                       {'jhb.geometry.method':'unknown'}, {'jhb.border_px':-1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_material_config({'use_jhb_model':True, **change})
        with self.assertRaises(MaterialModelError) as ctx:
            load_material_model(parse_material_config({'use_jhb_model':True,'jhb.model_path':'/missing/model.bin'}))
        self.assertEqual(ctx.exception.status,'MODEL_LOAD_FAILED')

    def test_raw_head_decode_and_classwise_nms(self):
        outs = [np.full((1,384//s,640//s,c), -80, np.float32)
                for s in (8,16,32) for c in (2,64)]
        outs[0][0,10,12,1] = 8
        distribution = outs[1][0,10,12].reshape(4,16)
        distribution[:,2] = 80
        boxes,scores,classes = decode(outs,.25)
        np.testing.assert_allclose(boxes,[[84,68,116,100]])
        self.assertEqual(classes.tolist(),[1])
        self.assertGreater(scores[0],.99)
        boxes = np.array([[10,10,30,30],[10,10,30,30],[10,10,30,30]],float)
        self.assertEqual(class_nms(boxes,np.array([.9,.8,.7]),np.array([0,0,1]),.45),[0,2])
        with self.assertRaises(ValueError):
            decode(outs[:-1],.25)

    def test_solid_cap_contour_uses_shared_ring_geometry(self):
        image = np.full((160,160,3),240,np.uint8)
        cv2.circle(image,(80,80),50,(30,30,30),-1)
        config = GeometryConfig(min_radius_ratio=.35)
        result = RingGeometry(config,contour_boundary='outer').detect(image,(30,30,130,130))
        self.assertIsNotNone(result.center_px)
        np.testing.assert_allclose(result.center_px,(80,80),atol=1)
        self.assertGreater(result.support['min_supported_sectors'],7)
        # The normal ring path still searches enclosed holes, not solid cap contours.
        self.assertIsNone(RingGeometry(config).detect(image,(30,30,130,130)).center_px)


@unittest.skipUnless(ONNX.is_file(),'JHB model package not installed')
class JhbPackagedModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(2)
        cls.p = parse_material_config({'use_jhb_model':True,'jhb.model_path':str(ONNX)})
        cls.model = load_material_model(cls.p)
        cls.frame = cv2.imread(str(ROOT/'detector/selftest.jpg'))
        cls.detections = cls.model.infer(cls.frame)

    def test_packaged_selftest_centers_and_full_pipeline(self):
        expected = json.loads((ROOT/'detector/selftest.json').read_text())
        caps = [d for d in self.detections if d.label == 'cap_top']
        self.assertEqual(len(caps),3)
        centers = np.array([[(d.bbox_xyxy[0]+d.bbox_xyxy[2])/8,
                             (d.bbox_xyxy[1]+d.bbox_xyxy[3])/8] for d in caps])
        for point in expected['caps']:
            self.assertLess(np.linalg.norm(centers-point,axis=1).min(),expected['tolerance_px'])
        original = self.frame.copy()
        result = MaterialDetector(self.p,self.model).detect(self.frame,7,1.5)
        self.assertEqual(len(result.observations),3)
        self.assertEqual(result.backend,'jhb')
        self.assertEqual({o.color_id for o in result.observations if o.valid},{3,4})
        self.assertTrue(any(o.status=='COLOR_REJECTED' for o in result.observations))
        for item in result.observations:
            self.assertIsNotNone(item.material_bbox_xyxy)
            if not item.valid:
                self.assertEqual(item.color_id, 0)
                self.assertIsNotNone(item.center_px)
                self.assertIn(item.center_source, ('bbox', 'circle'))
        np.testing.assert_array_equal(original,self.frame)
        json.dumps(result.to_dict(),allow_nan=False)

    def test_pairing_is_one_to_one_and_checks_truncation(self):
        box = (40,40,120,120)
        cap = JhbDetection('cap_top',.95,box)
        material = JhbDetection('material',.9,(30,30,160,170))
        pairs,rejected = self.model.pair_caps([cap,cap,material],(300,300,3))
        self.assertEqual(len(pairs),1)
        self.assertEqual(rejected[0]['reason'],'CAP_UNPAIRED')
        pairs,rejected = self.model.pair_caps([JhbDetection('cap_top',.99,(0,40,80,120)),material],(300,300,3))
        self.assertFalse(pairs)
        self.assertEqual(rejected[0]['reason'],'TRUNCATED')
        pairs,rejected = self.model.pair_caps([JhbDetection('cap_top',.5,box),material],(300,300,3))
        self.assertEqual(rejected[0]['reason'],'CAP_LOW_SCORE')

    def test_no_paired_caps_skips_white_balance_and_color_processing(self):
        detector = MaterialDetector(self.p, self.model)
        roi = (0, 0, self.frame.shape[1], self.frame.shape[0])
        result = detector._new_result(self.frame, 1, 0., roi)
        detector._combine_jhb(self.frame, [], roi, 0, result, 0)
        self.assertEqual(result.observations, [])
        self.assertEqual(result.stages_ms['color'], 0.)
        self.assertEqual(result.stages_ms['geometry'], 0.)
        self.assertNotIn('jhb_white_balance_gains', result.debug)

    def test_lab_six_classes_same_color_instances_and_rejection(self):
        image = np.zeros((160,160,3),np.uint8)
        for color, lab in enumerate(self.model.mu,1):
            bgr = cv2.cvtColor(lab.astype(np.float32).reshape(1,1,3),cv2.COLOR_Lab2BGR)[0,0]
            image[:] = np.rint(bgr*255).astype(np.uint8)
            fine, gains, _ = self.model.color_context(image,[])
            actual,probability,_,reason = self.model.classify(fine,image.shape,(80,80),50,np.ones(3))
            self.assertEqual((actual,reason),(color,'OK'))
            self.assertGreaterEqual(probability,.9)
        image[:] = 128
        fine,_,_ = self.model.color_context(image,[])
        actual,_,_,reason = self.model.classify(fine,image.shape,(80,80),50,np.ones(3))
        self.assertEqual(actual,0)
        self.assertNotEqual(reason,'OK')
        caps = [JhbDetection('cap_top',.95,(20,20,80,80)),JhbDetection('cap_top',.9,(140,20,200,80))]
        materials = [JhbDetection('material',.9,(10,10,90,100)),JhbDetection('material',.9,(130,10,210,100))]
        self.assertEqual(len(self.model.pair_caps(caps+materials,(250,250,3))[0]),2)

    def test_roi_offsets_and_no_circle_fallback_disabled(self):
        p = {**self.p,'top.allow_bbox_fallback':False,'jhb.geometry.method':'hough',
             'jhb.geometry.hough_param2':10000.0}
        detector = MaterialDetector(p,load_material_model(p))
        result = detector.detect(self.frame)
        self.assertTrue(result.observations)
        self.assertFalse(any(o.valid for o in result.observations))
        self.assertTrue(all(o.center_px is None for o in result.observations))
        result = MaterialDetector(self.p,self.model).detect(self.frame,search_roi=[100,100,2800,700],target_color_id=4)
        self.assertTrue(any(o.valid and o.color_id==4 for o in result.observations))
        for item in result.observations:
            self.assertGreaterEqual(item.bbox_xyxy[0],100)
            self.assertGreaterEqual(item.bbox_xyxy[1],100)
            self.assertTrue(not item.valid or item.color_id==4)

    def test_paired_solid_caps_restore_circle_and_ellipse_offsets(self):
        frame = np.full((300,340,3),240,np.uint8)
        lab = self.model.mu[3].astype(np.float32).reshape(1,1,3)
        bgr = np.rint(cv2.cvtColor(lab,cv2.COLOR_Lab2BGR)[0,0]*255).astype(np.uint8)
        cv2.circle(frame,(110,130),50,tuple(int(v) for v in bgr),-1)
        cv2.circle(frame,(250,130),50,tuple(int(v) for v in bgr),-1)
        boxes = [JhbDetection('cap_top',.95,(60,80,160,180)),
                 JhbDetection('material',.9,(50,70,170,220)),
                 JhbDetection('cap_top',.95,(200,80,300,180)),
                 JhbDetection('material',.9,(190,70,310,220))]
        detector = MaterialDetector(self.p,self.model)
        roi = (20,30,360,330)
        result = detector._new_result(frame,1,0,roi)
        detector._combine_jhb(frame,boxes,roi,4,result,4)
        self.assertEqual(len(result.observations),2)
        for item,expected in zip(result.observations,[(130,160),(270,160)]):
            self.assertEqual((item.color_id,item.status,item.center_source),(4,'OK','circle'))
            np.testing.assert_allclose(item.circle_center_px,expected,atol=1)
            np.testing.assert_allclose(item.ellipses_px[0][0],expected,atol=1)
            self.assertGreaterEqual(item.circle_support['min_supported_sectors'],8)

    def test_rejected_color_keeps_refined_center_and_roi_offset(self):
        frame = np.full((300,340,3), 240, np.uint8)
        cv2.circle(frame, (110,130), 50, (128,128,128), -1)
        boxes = [JhbDetection('cap_top', .95, (60,80,160,180)),
                 JhbDetection('material', .9, (50,70,170,220))]
        detector = MaterialDetector(self.p, self.model)
        roi = (20,30,360,330)
        result = detector._new_result(frame, 1, 0, roi)
        detector._combine_jhb(frame, boxes, roi, 4, result, 4)
        item, = result.observations
        self.assertFalse(item.valid)
        self.assertEqual((item.color_id, item.status, item.center_source), (0, 'COLOR_REJECTED', 'circle'))
        np.testing.assert_allclose(item.center_px, (130,160), atol=1)
        self.assertEqual(item.center_px, item.circle_center_px)


if __name__ == '__main__':
    unittest.main()
