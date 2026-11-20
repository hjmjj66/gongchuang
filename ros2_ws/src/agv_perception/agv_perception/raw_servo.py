"""Raw gantry PID with a short-horizon estimate of relative pixel motion."""

import math

import cv2
import numpy as np

from .work_control import PID


class RawGantryServo:
    def __init__(self, config):
        self.p = config['motion_prediction']
        self.gate = config['grasp_gate']
        c = config['gantry_servo']
        self.pids = [PID(c['yaw_pid'], c['max_yaw_rad_s']), PID(c['fb_pid'], c['max_fb_rpm'])]
        self.reset()

    def reset(self):
        for pid in self.pids:
            pid.reset()
        self.observed_at = None
        self.error = (0., 0.)
        self.velocity = (0., 0.)
        self.active = False
        self.diagnostic = {}
        self.filter = None

    def observe(self, error, captured_at):
        dt = None if self.observed_at is None else captured_at-self.observed_at
        if self.p['enabled'] and dt is not None and dt > self.p['max_horizon_s']:
            self.reset()
        self.error = tuple(error)
        if self.p['enabled']:
            self._correct(error, dt)
        else:
            self.velocity = tuple(-pid.update(e, captured_at) for pid, e in zip(self.pids, error))
        self.observed_at = captured_at
        self.active = True

    def update(self, now):
        if self.observed_at is None:
            return self.velocity
        age = now-self.observed_at
        if self.p['enabled'] and age > self.p['max_horizon_s']:
            error = self.error
            self.reset()
            self.diagnostic = dict(error_uv=error, filtered_error_uv=None, pixel_velocity_px_s=None,
                                   predicted_error_uv=None, prediction_horizon_s=0., frame_age_s=age,
                                   prediction_enabled=True, prediction_active=False, yaw_wz=0., fb_rpm=0.,
                                   wait_reason='超过短时预测期限，停轴等待新帧')
            return self.velocity
        filtered = self.error
        pixel_velocity = (0., 0.)
        horizon = 0.
        if self.p['enabled']:
            state = self.filter.statePost[:, 0]
            filtered = tuple(float(v) for v in state[:2])
            pixel_velocity = tuple(float(v) for v in state[2:])
            horizon = min(age+self.p['extra_delay_s'], self.p['max_horizon_s'])
        # Projection never advances the posterior: the next measurement has its own timestamp.
        predicted = tuple(e+v*horizon for e, v in zip(filtered, pixel_velocity))
        if self.p['enabled']:
            self.velocity = tuple(-pid.update(e, now) for pid, e in zip(self.pids, predicted))
        self.diagnostic = dict(error_uv=self.error, filtered_error_uv=filtered,
                               pixel_velocity_px_s=pixel_velocity, predicted_error_uv=predicted,
                               prediction_horizon_s=horizon, frame_age_s=age,
                               prediction_enabled=self.p['enabled'], prediction_active=self.p['enabled'] and self.active,
                               yaw_wz=self.velocity[0], fb_rpm=self.velocity[1])
        return self.velocity

    @property
    def aligned(self):
        c = self.gate
        return (self.active and math.hypot(*self.error) <= c['tolerance_px'] and
                abs(self.velocity[0]) <= c['small_yaw_rad_s'] and
                abs(self.velocity[1]) <= c['small_fb_rpm'])

    def _correct(self, error, dt):
        if self.filter is None:
            k = self.filter = cv2.KalmanFilter(4, 2, 0, cv2.CV_64F)
            k.measurementMatrix = np.eye(2, 4, dtype=np.float64)
            k.measurementNoiseCov = np.eye(2)*self.p['measurement_std_px']**2
            k.errorCovPost = np.diag(np.array(
                [self.p['measurement_std_px']**2]*2+
                [self.p['initial_velocity_std_px_s']**2]*2, dtype=np.float64))
            k.statePost = np.array([[error[0]], [error[1]], [0.], [0.]], dtype=np.float64)
            return
        k = self.filter
        k.transitionMatrix = np.array([[1., 0., dt, 0.], [0., 1., 0., dt],
                                       [0., 0., 1., 0.], [0., 0., 0., 1.]])
        g = np.array([[dt*dt/2, 0.], [0., dt*dt/2], [dt, 0.], [0., dt]])
        k.processNoiseCov = (g @ g.T)*self.p['acceleration_std_px_s2']**2
        k.predict()
        k.correct(np.asarray(error, dtype=np.float64).reshape(2, 1))
