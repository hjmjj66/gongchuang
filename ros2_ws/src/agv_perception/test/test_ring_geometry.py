"""Exercise OpenCV on generated rings and inspect the actual fusion algorithm."""

from dataclasses import replace
import math
import unittest

import cv2
import numpy as np

from agv_perception.ring_geometry import CircleCandidate, GeometryConfig, RingGeometry


def ring_image(center=(190, 150), radii=(32, 53, 73), digit='1'):
    frame = np.full((320, 400, 3), 245, dtype=np.uint8)
    for radius in radii:
        cv2.circle(frame, center, radius, (10, 10, 10), 3, cv2.LINE_AA)
    if digit:
        cv2.putText(frame, digit, (center[0] - 6, center[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (10, 10, 10), 2, cv2.LINE_AA)
    return frame


class GeometryTests(unittest.TestCase):
    def test_real_hough_single_circle_with_center_digit(self):
        frame = ring_image(center=(190, 150), radii=(73,), digit='2')
        geometry = RingGeometry()
        result = geometry.detect(frame[60:240, 100:280], (10, 10, 170, 170))
        self.assertEqual(result.reason, 'OK')
        self.assertLess(math.dist(result.center_px, (90, 90)), 4.0)
        self.assertGreaterEqual(result.support['edge_ratio'], geometry.config.min_edge_support)
        self.assertTrue(result.selected)

    def test_real_hough_concentric_rings_both_preprocessing_modes(self):
        frame = ring_image()
        for mode in ('gray', 'binary_morphology'):
            with self.subTest(mode=mode):
                geometry = RingGeometry(replace(GeometryConfig(), preprocess=mode))
                result = geometry.detect(frame[57:243, 97:283], (13, 13, 173, 173))
                self.assertEqual(result.reason, 'OK')
                self.assertLess(math.dist(result.center_px, (93, 93)), 4.0)
                self.assertGreaterEqual(len(result.radii_px), 1)

    def test_white_image_and_digit_alone_have_no_trusted_circle(self):
        geometry = RingGeometry()
        for digit in ('', '2'):
            with self.subTest(digit=digit):
                frame = ring_image(radii=(), digit=digit)
                result = geometry.detect(frame[60:240, 100:280], (10, 10, 170, 170))
                self.assertIsNone(result.center_px)

    def test_local_noise_keeps_center(self):
        frame = ring_image(radii=(73,), digit='3')
        random = np.random.default_rng(5)
        for x, y in random.integers((110, 70), (270, 230), size=(30, 2)):
            cv2.circle(frame, (int(x), int(y)), 1, (0, 0, 0), -1)
        result = RingGeometry().detect(frame[57:243, 97:283], (13, 13, 173, 173))
        self.assertEqual(result.reason, 'OK')
        self.assertLess(math.dist(result.center_px, (93, 93)), 4.0)

    def test_real_off_center_circle_rejected_by_center_constraint(self):
        frame = np.full((200, 200, 3), 240, np.uint8)
        cv2.circle(frame, (40, 100), 26, (0, 0, 0), 3, cv2.LINE_AA)
        result = RingGeometry().detect(frame, (0, 0, 200, 200))
        self.assertIsNone(result.center_px)
        self.assertTrue(any(c.reason == 'CENTER_OUTSIDE_RANGE' for c in result.candidates))

    def test_edge_support_requires_distributed_arc(self):
        edges = np.zeros((200, 200), np.uint8)
        cv2.ellipse(edges, (100, 100), (60, 60), 0, 0, 80, 255, 2)
        geometry = RingGeometry()
        support, sectors, alignment = geometry._edge_support(edges, 100, 100, 60)
        self.assertLess(support, geometry.config.min_edge_support)
        self.assertLess(sectors, geometry.config.min_supported_sectors)

    def test_circle_edge_alignment_prefers_the_actual_radius(self):
        edges = np.zeros((200, 200), np.uint8)
        cv2.circle(edges, (100, 100), 60, 255, 1, cv2.LINE_8)
        geometry = RingGeometry()
        true_support, _, true_alignment = geometry._edge_support(edges, 100, 100, 60)
        shifted_support, _, shifted_alignment = geometry._edge_support(edges, 100, 100, 62)
        self.assertGreater(true_support, geometry.config.min_edge_support)
        self.assertGreater(shifted_support, geometry.config.min_edge_support)
        self.assertGreater(true_alignment, shifted_alignment)

    def test_weighted_fusion_and_duplicate_edge_not_counted_twice(self):
        candidates = (
            CircleCandidate(100, 100, 60, 1.0, 12),
            CircleCandidate(102, 100, 40, 0.8, 10),
            CircleCandidate(101, 100, 60.5, 0.7, 9),
        )
        result = RingGeometry().fuse_candidates(candidates, (100, 100), 160)
        self.assertEqual(result.support['circle_count'], 2)
        self.assertAlmostEqual(result.center_px[0], (100 + 102 * 0.8) / 1.8)
        self.assertAlmostEqual(result.center_px[1], 100)
        self.assertEqual(result.radii_px, (60, 40))

    def test_center_chains_do_not_form_one_group(self):
        config = replace(GeometryConfig(), merge_distance_ratio=0.05,
                         group_ambiguity_margin=0.0)
        candidates = (
            CircleCandidate(100, 100, 60, 1.0, 12),
            CircleCandidate(104, 100, 40, 0.9, 12),
            CircleCandidate(108, 100, 20, 0.8, 12),
        )
        result = RingGeometry(config).fuse_candidates(candidates, (100, 100), 100)
        self.assertEqual(result.support['circle_count'], 2)
        self.assertLess(result.center_px[0], 104)

    def test_two_equally_supported_groups_are_ambiguous(self):
        candidates = (
            CircleCandidate(85, 100, 40, 0.95, 12),
            CircleCandidate(115, 100, 40, 0.95, 12),
        )
        result = RingGeometry().fuse_candidates(candidates, (100, 100), 160)
        self.assertEqual(result.reason, 'AMBIGUOUS_CIRCLE_GROUPS')
        self.assertIsNone(result.center_px)

    def test_invalid_geometry_config_rejected_at_entry(self):
        for changes in ({'blur_kernel': 4}, {'min_edge_support': 0.0},
                        {'min_radius_ratio': 0.7}, {'angle_sectors': 0},
                        {'open_iterations': -1}, {'hough_param2': float('nan')}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(GeometryConfig(), **changes)


if __name__ == '__main__':
    unittest.main()
