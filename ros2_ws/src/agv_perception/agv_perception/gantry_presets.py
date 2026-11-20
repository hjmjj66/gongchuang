"""Calibrated approach poses and feedback-based motion before visual refinement."""

import math

from .work_control import PID


DEFAULTS = dict(positions={'1': None, '3': None})


def validate_presets(config):
    if config is None:
        return
    positions = config['positions']
    if not isinstance(positions, dict) or set(positions) != {'1', '3'}:
        raise ValueError('gantry_presets.positions 须包含位置1和3')
    for point in positions.values():
        if point is None:
            continue
        if (not isinstance(point, dict) or set(point) != {'fb_pulses', 'yaw_rad'} or
                type(point['fb_pulses']) is not int or not -0x80000000 <= point['fb_pulses'] <= 0x7FFFFFFF or
                type(point['yaw_rad']) not in (int, float) or not math.isfinite(point['yaw_rad'])):
            raise ValueError('机构初值须为实际 fb_pulses（int32 有符号脉冲）和 yaw_rad（rad）；旧毫米点位不可直接使用')


class PresetMove:
    def __init__(self, config, target, now, feedback_timeout_s):
        self.p, self.target, self.started_at = config, target, now
        self.feedback_timeout_s = feedback_timeout_s
        self.timed_out = False
        self.pids = [PID(config['yaw_pid'], config['max_yaw_rad_s']),
                     PID(config['length_pid'], config['max_fb_rpm'])]

    def update(self, now, gantry):
        if gantry is None or not 0 <= now-gantry.received_at <= self.feedback_timeout_s:
            raise RuntimeError('机构初值预定位缺少新鲜位置反馈')
        self.timed_out = now-self.started_at >= self.p['timeout_s']
        el = self.target['fb_pulses']-gantry.fb_pulses
        et = self.target['yaw_rad']-gantry.yaw_rad
        good = self.timed_out or (abs(el) <= self.p['length_tolerance_pulses'] and abs(et) <= self.p['angle_tolerance_rad'])
        velocity = ((0., 0.) if good else
                    (self.pids[0].update(et, gantry.received_at),
                     self.pids[1].update(el, gantry.received_at)))
        return good, velocity, dict(target_rt=self.target, current_rt=dict(fb_pulses=gantry.fb_pulses,
                                  yaw_rad=gantry.yaw_rad), error_rt=(el, et),
                                  motion_outcome='TIMEOUT_SKIPPED' if self.timed_out else 'RUNNING')
