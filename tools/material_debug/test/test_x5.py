"""Opt-in real BPU tests, never substitute mock boxes for model predictions."""

import json
import os
import time
from pathlib import Path
import unittest

import cv2
import numpy as np

from agv_perception.material_config import read_material_config,parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_model import load_material_model
from server import DEFAULT_CONFIG


@unittest.skipUnless(os.environ.get('AGV_TEST_X5_MATERIAL') == '1','set AGV_TEST_X5_MATERIAL=1 on X5')
class RealModelTests(unittest.TestCase):
    def test_combine_real_models_and_stage_timings(self):
        p = parse_material_config({**read_material_config(DEFAULT_CONFIG),
                                   'test_combine':True, 'use_jhb_model':False})
        detector = MaterialDetector(p,load_material_model(p))
        image = cv2.imread('/root/material_deploy_v1/export/upper_top/test_images/check.jpg')
        detector.detect(image)  # Warm the two runtimes before measuring.
        started = time.perf_counter()
        results = [detector.detect(image, frame_seq=i+1) for i in range(5)]
        elapsed = time.perf_counter()-started
        for result in results:
            self.assertIsNone(result.error)
            self.assertTrue(any(o.valid for o in result.observations))
            self.assertTrue(all(o.color_id in (0,1,2,4) for o in result.observations))
            self.assertTrue(all(o.identity_source=='yolo_jhb' for o in result.observations))
            self.assertNotIn('color',result.stages_ms)
        stages = {name:round(sum(r.stages_ms[name] for r in results)/len(results),2)
                  for name in ('color_yolo','jhb_yolo','matching','geometry')}
        print(f'\nReal X5 test_combine: {len(results)/elapsed:.1f} FPS, stages_ms={stages}')

    def test_two_models_six_labels_original_coordinates(self):
        p=read_material_config(DEFAULT_CONFIG)
        images=[('/root/material_deploy_v1/export/upper_top/test_images/check.jpg',{1,2,4}),
                ('/root/material_deploy_v1/check_black_blues.jpg',{3,5,6})]
        for kind in (1,2):
            config=parse_material_config({**p,'material_model':kind})
            model=load_material_model(config)
            detector=MaterialDetector(config,model)
            for path,expected in images:
                with self.subTest(model=kind,image=path):
                    image=cv2.imread(path)
                    original=image.copy()
                    result=detector.detect(image,12,3.5)
                    self.assertEqual({o.color_id for o in result.observations},expected)
                    self.assertEqual(result.image_size,(image.shape[1],image.shape[0]))
                    self.assertEqual(result.status,'OK')
                    self.assertTrue(all(o.valid for o in result.observations))
                    for o in result.observations:
                        self.assertLess(o.bbox_xyxy[0],image.shape[1])
                        self.assertLess(o.bbox_xyxy[1],image.shape[0])
                        if kind==2:
                            self.assertEqual(o.center_px,o.bbox_center_px)
                            self.assertIsNone(o.circle_center_px)
                    np.testing.assert_array_equal(image,original)
                    json.dumps(result.to_dict(),allow_nan=False)
            # A shifted crop verifies that model coordinates are restored to crop dimensions.
            image=cv2.imread(images[0][0])
            roi=(250,200,850,700)
            raw=model.infer(image[roi[1]:roi[3],roi[0]:roi[2]])
            result=detector.detect(image,search_roi=list(roi))
            for o in result.observations:
                match=next(b for b in raw if b.color_id==o.color_id)
                np.testing.assert_allclose(o.bbox_xyxy,np.array(match.bbox_xyxy)+(250,200,250,200),atol=.01)


if __name__ == '__main__':
    unittest.main()
