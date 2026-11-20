"""Exercise the actual estimator and PID against timestamped pixel trajectories."""

from copy import deepcopy
from pathlib import Path
import unittest

import numpy as np
import yaml

from agv_perception.raw_control import validate_raw_config
from agv_perception.raw_servo import RawGantryServo
from agv_perception.work_control import PID
from test_raw_control import calibrated_raw_config


class RawServoTests(unittest.TestCase):
    def setUp(self):
        self.config = calibrated_raw_config()
        self.config['motion_prediction']['enabled'] = True
        self.config['gantry_servo'].update(max_yaw_rad_s=1.,max_fb_rpm=50.,
                                         yaw_pid=dict(kp=.001,ki=0.,kd=0.),
                                         fb_pid=dict(kp=.01,ki=0.,kd=0.))
        self.servo = RawGantryServo(self.config)

    def test_first_measurement_initializes_at_observed_position(self):
        self.servo.observe((30.,-10.),1.)
        self.assertEqual(self.servo.update(1.05),(-.03,.1))
        self.assertEqual(self.servo.diagnostic['pixel_velocity_px_s'],(0.,0.))
        self.assertEqual(self.servo.diagnostic['filtered_error_uv'],(30.,-10.))

    def test_integer_noise_settings_allow_repeated_predictions(self):
        self.config['motion_prediction'].update(measurement_std_px=2,
                                                initial_velocity_std_px_s=100,
                                                acceleration_std_px_s2=500)
        validate_raw_config(self.config)
        for stamp in (1., 1.04, 1.08):
            self.servo.observe((30., -10.), stamp)
            yaw, fb = self.servo.update(stamp+.02)
            self.assertAlmostEqual(yaw, -.03)
            self.assertAlmostEqual(fb, .1)
            np.testing.assert_allclose(self.servo.diagnostic['pixel_velocity_px_s'], (0., 0.), atol=1e-10)

    def test_irregular_frame_times_and_delay_predict_constant_velocity(self):
        self.config['motion_prediction']['extra_delay_s'] = .02
        stamp = 1.
        for dt in ([.02,.045,.03,.06]*20):
            stamp += dt
            self.servo.observe((20.+80.*stamp,30.-35.*stamp),stamp)
            self.servo.update(stamp+.06)
        d = self.servo.diagnostic
        self.assertAlmostEqual(d['pixel_velocity_px_s'][0],80.,delta=.01)
        self.assertAlmostEqual(d['pixel_velocity_px_s'][1],-35.,delta=.01)
        self.assertAlmostEqual(d['predicted_error_uv'][0],20.+80.*(stamp+.08),delta=.01)
        self.assertAlmostEqual(d['predicted_error_uv'][1],30.-35.*(stamp+.08),delta=.01)
        self.assertAlmostEqual(d['prediction_horizon_s'],.08)

    def test_stationary_noise_is_reduced_without_hiding_raw_measurement(self):
        rng = np.random.default_rng(19)
        raw, filtered = [], []
        for i in range(250):
            error = np.array([20.,-15.])+rng.normal(0.,2.,2)
            self.servo.observe(error,1.+i/30.)
            self.servo.update(1.+i/30.)
            if i > 30:
                raw.append(error-[20.,-15.])
                filtered.append(np.array(self.servo.diagnostic['filtered_error_uv'])-[20.,-15.])
            self.assertEqual(self.servo.diagnostic['error_uv'],tuple(error))
        self.assertLess(np.mean(np.square(filtered)),.7*np.mean(np.square(raw)))

    def test_projection_does_not_advance_posterior_or_double_count_latency(self):
        for i in range(30):
            self.servo.observe((10.+i,0.),1.+i*.04)
            self.servo.update(1.+i*.04+.01)
        stamp = self.servo.observed_at
        state, covariance = self.servo.filter.statePost.copy(), self.servo.filter.errorCovPost.copy()
        self.servo.update(stamp+.02)
        first = self.servo.diagnostic['predicted_error_uv'][0]
        speed = self.servo.diagnostic['pixel_velocity_px_s'][0]
        self.servo.update(stamp+.04)
        self.assertAlmostEqual(self.servo.diagnostic['predicted_error_uv'][0]-first,speed*.02)
        np.testing.assert_array_equal(self.servo.filter.statePost,state)
        np.testing.assert_array_equal(self.servo.filter.errorCovPost,covariance)

    def test_horizon_caps_extra_delay_and_expiry_stops_and_resets(self):
        self.config['motion_prediction']['extra_delay_s'] = .1
        for i in range(8):
            self.servo.observe((10.+i,5.),1.+i*.04)
            self.servo.update(1.+i*.04)
        stamp = self.servo.observed_at
        self.servo.update(stamp+.1)
        self.assertEqual(self.servo.diagnostic['prediction_horizon_s'],.15)
        self.assertEqual(self.servo.update(stamp+.151),(0.,0.))
        self.assertFalse(self.servo.active)
        self.assertTrue(all(p.integral == 0. for p in self.servo.pids))
        self.assertIsNone(self.servo.filter)
        self.servo.observe((0.,0.),stamp+.2)
        self.servo.update(stamp+.2)
        self.assertEqual(self.servo.diagnostic['pixel_velocity_px_s'],(0.,0.))

    def test_observation_gap_reinitializes_without_a_control_tick(self):
        self.servo.observe((100.,20.),1.)
        self.servo.update(1.)
        self.servo.observe((0.,0.),1.3)
        self.assertEqual(self.servo.update(1.3),(0.,0.))
        self.assertTrue(self.servo.aligned)
        self.assertEqual(self.servo.diagnostic['pixel_velocity_px_s'],(0.,0.))

    def test_abrupt_stop_eventually_settles_and_commands_keep_existing_limits(self):
        for i in range(30):
            self.servo.observe((i*5.,-i*100.),1.+i*.04)
            yaw, fb = self.servo.update(1.+i*.04+.05)
            self.assertLessEqual(abs(yaw),1.)
            self.assertLessEqual(abs(fb),50.)
        for i in range(50):
            self.servo.observe((0.,0.),2.2+i*.04)
            self.servo.update(2.2+i*.04)
        self.assertTrue(self.servo.aligned)
        self.assertLess(np.linalg.norm(self.servo.diagnostic['pixel_velocity_px_s']),.01)

    def test_disabled_matches_existing_pid_including_frame_hold(self):
        self.config['motion_prediction']['enabled'] = False
        c = self.config['gantry_servo']
        for key in ('yaw_pid','fb_pid'):
            c[key].update(ki=.0001,kd=.0003)
        servo = RawGantryServo(self.config)
        reference = [PID(c['yaw_pid'],c['max_yaw_rad_s']),PID(c['fb_pid'],c['max_fb_rpm'])]
        for t, error in ((1.,(10.,20.)),(1.05,(8.,18.)),(1.12,(-5.,0.))):
            expected = tuple(-p.update(e,t) for p,e in zip(reference,error))
            servo.observe(error,t)
            self.assertEqual(servo.update(t+.03),expected)
            self.assertEqual(servo.update(t+.1),expected)
        self.assertIsNone(servo.filter)

    def test_delayed_noisy_motion_predicts_closer_than_raw_measurements(self):
        rng = np.random.default_rng(23)
        raw_errors, predicted_errors = [], []
        for i in range(200):
            stamp = 1.+i/30.
            measured = np.array([80.*stamp,-30.*stamp])+rng.normal(0.,2.,2)
            self.servo.observe(measured,stamp)
            self.servo.update(stamp+.08)
            if i > 20:
                truth = np.array([80.*(stamp+.08),-30.*(stamp+.08)])
                raw_errors.append(measured-truth)
                predicted_errors.append(np.array(self.servo.diagnostic['predicted_error_uv'])-truth)
        self.assertLess(np.mean(np.square(predicted_errors)),.7*np.mean(np.square(raw_errors)))

    def test_prediction_configuration_and_production_debug_switches(self):
        validate_raw_config(self.config)
        for key,value in (('enabled',1),('measurement_std_px',0.),('acceleration_std_px_s2',float('nan')),
                          ('extra_delay_s',-.01),('extra_delay_s',.16),('max_horizon_s',.6)):
            p = deepcopy(self.config)
            p['motion_prediction'][key] = value
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                validate_raw_config(p)
        production = yaml.safe_load((Path(__file__).parents[1]/'config/raw.yaml').read_text())['raw']
        self.assertTrue(production['motion_prediction']['enabled'])


if __name__ == '__main__':
    unittest.main()
