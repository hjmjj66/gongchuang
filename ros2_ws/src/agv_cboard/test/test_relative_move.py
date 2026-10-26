"""Relative axis geometry, signed progress, and approach speed."""
import math
import unittest

from agv_cboard.relative_move import RelativeMove


class RelativeMoveTests(unittest.TestCase):
    def test_axes_at_rotated_nonzero_origin(self):
        origin = (2., 3., math.pi/2)
        left = (1.7, 3., math.pi/2)
        forward = (2., 3.3, math.pi/2)
        x = RelativeMove('x', .3, .05, .005)
        y = RelativeMove('y', .3, .05, .005)
        self.assertAlmostEqual(x.displacement(origin, forward), .3)
        self.assertAlmostEqual(y.displacement(origin, left), .3)
        self.assertAlmostEqual(x.displacement(origin, left), 0.)
        self.assertAlmostEqual(y.displacement(origin, forward), 0.)

    def test_negative_targets_wrong_direction_and_overshoot(self):
        for axis, index in (('x', 0), ('y', 1), ('yaw', 2)):
            move = RelativeMove(axis, -30., 5., .1)
            self.assertEqual(move.command(0.)[index], -5.)
            self.assertGreater(move.remaining(1.), 30.)
            self.assertEqual(move.command(-30.1), (0., 0., 0.))
            self.assertEqual(move.command(30.)[index], -5.)

    def test_unwrapped_angles_and_slowdown(self):
        move = RelativeMove('yaw', 450., 10., 1.)
        self.assertAlmostEqual(move.displacement((0, 0, 4*math.pi),
                                                (0, 0, 6.5*math.pi)), 450.)
        self.assertLess(move.command(445.)[2], move.command(0.)[2])
        self.assertEqual(move.command(449.), (0., 0., 0.))

    def test_reject_invalid_parameters(self):
        for target, speed, tolerance in ((0, .05, .005), (.1, 0, .005),
                                          (.1, .05, 0), (float('nan'), .05, .005)):
            with self.assertRaises(ValueError):
                RelativeMove('x', target, speed, tolerance)
