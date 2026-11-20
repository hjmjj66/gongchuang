import unittest

import cv2
import numpy as np

from agv_perception.material_config import parse_material_config
from agv_perception.material_geometry import TopCircleLocator


class TopGeometryTests(unittest.TestCase):
    def test_solid_circle_real_hough(self):
        frame=np.full((130,130,3),25,np.uint8)
        cv2.circle(frame,(65,65),45,(220,220,220),-1)
        r=TopCircleLocator(parse_material_config({})).detect(frame,(19,19,111,111))
        self.assertEqual(r.reason,'OK')
        np.testing.assert_allclose(r.center,(65,65),atol=2)
        self.assertGreater(r.support,.8)
        self.assertAlmostEqual(r.radius,45,delta=3)

    def test_blank_and_wrong_radius_have_no_circle(self):
        blank=np.full((130,130,3),140,np.uint8)
        locator=TopCircleLocator(parse_material_config({}))
        self.assertEqual(locator.detect(blank,(19,19,111,111)).reason,'NO_CIRCLE')
        cv2.circle(blank,(65,65),8,(20,20,20),-1)
        self.assertIsNone(locator.detect(blank,(19,19,111,111)).center)

    def test_strict_offset_rejects_wrong_circle(self):
        frame=np.full((160,160,3),25,np.uint8)
        cv2.circle(frame,(91,80),40,(220,220,220),-1)
        p=parse_material_config({'top_circle.max_center_offset_ratio':.01})
        r=TopCircleLocator(p).detect(frame,(39,39,121,121))
        self.assertEqual(r.reason,'NO_RELIABLE_CIRCLE')
        self.assertIsNone(r.center)

    def test_two_distinct_supported_circles_are_ambiguous(self):
        frame=np.full((180,240,3),25,np.uint8)
        cv2.circle(frame,(65,90),35,(220,220,220),2)
        cv2.circle(frame,(175,90),35,(220,220,220),2)
        p=parse_material_config({'top_circle.min_radius_ratio':.18,'top_circle.max_radius_ratio':.25,
                                 'top_circle.max_center_offset_ratio':.5,'top_circle.ambiguity_margin':1.0})
        r=TopCircleLocator(p).detect(frame,(20,10,220,170))
        self.assertEqual(r.reason,'AMBIGUOUS_CIRCLES')
        self.assertIsNone(r.center)


if __name__ == '__main__':
    unittest.main()
