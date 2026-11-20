import unittest

import cv2
import numpy as np

from agv_perception.material_config import COLORS,parse_material_config
from agv_perception.material_detector import MaterialDetector


def hsv_image(shape=(220,400)):
    image = np.zeros((*shape,3),np.uint8)
    image[:]=(0,0,255)
    return image


class HsvTests(unittest.TestCase):
    def test_actual_blue_lightblue_overlap_is_rejected_before_filter(self):
        hsv=hsv_image()
        cv2.circle(hsv,(100,100),30,(100,130,100),-1)
        frame=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        d=MaterialDetector(parse_material_config({'material_backend':1}))
        r=d.detect(frame,target_color_id=3)
        self.assertEqual(len(r.observations),1)
        o=r.observations[0]
        self.assertEqual(o.status,'COLOR_AMBIGUOUS')
        self.assertIn(6,o.conflicting_colors)
        self.assertIsNone(o.center_px)
        self.assertGreater(o.color_overlap_ratio,.99)

    def test_six_controlled_colors_and_two_red_intervals(self):
        hsv=hsv_image((200,520))
        settings={'material_backend':1}
        samples=[(0,200,150),(25,200,150),(110,200,150),(55,200,150),(0,0,25),(85,200,150)]
        for i,(color,pixel) in enumerate(zip(COLORS,samples)):
            h,s,v=pixel
            settings[f'hsv.{color}_ranges']=[max(0,h-1),max(0,s-1),v-1,min(179,h+1),min(255,s+1),v+1]
            cv2.circle(hsv,(45+80*i,75),22,pixel,-1)
        settings['hsv.red_ranges'] += [174,199,149,179,201,151]
        cv2.circle(hsv,(45,150),18,(178,200,150),-1)
        frame=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        r=MaterialDetector(parse_material_config(settings)).detect(frame)
        self.assertEqual([o.color_id for o in r.observations],[1,1,2,3,4,5,6])
        self.assertTrue(all(o.valid and o.center_source == 'contour_centroid' for o in r.observations))
        self.assertTrue(all(o.yolo_confidence is None for o in r.observations))

    def test_intersecting_boxes_without_foreground_overlap_are_not_ambiguous(self):
        hsv=hsv_image()
        # Nested boxes, disjoint foreground: red donut and a yellow disk in its hole.
        cv2.circle(hsv,(100,100),60,(0,200,150),-1)
        cv2.circle(hsv,(100,100),40,(0,0,255),-1)
        cv2.circle(hsv,(100,100),25,(25,170,150),-1)
        r=MaterialDetector(parse_material_config({'material_backend':1})).detect(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR))
        self.assertEqual([o.color_id for o in r.observations],[1,2])
        self.assertTrue(all(o.valid for o in r.observations))

    def test_small_noise_missing_color_and_boundary(self):
        hsv=hsv_image()
        cv2.circle(hsv,(0,100),25,(0,200,150),-1)
        cv2.circle(hsv,(100,100),2,(0,200,150),-1)
        d=MaterialDetector(parse_material_config({'material_backend':1,'hsv.black_ranges':[]}))
        r=d.detect(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR))
        self.assertEqual(len(r.observations),1)
        self.assertEqual(r.observations[0].status,'TRUNCATED')
        self.assertEqual(r.status,'CONFIGURATION_INCOMPLETE')
        self.assertTrue(r.warnings)


if __name__ == '__main__':
    unittest.main()
