"""Planar work controller retained for storage placement and stacking."""

import math
from pathlib import Path

from .work_control import Observation, PID, Pose, clamp, line_slope, return_velocity


def validate_work_config(p, arm_commands, servo_sections):
    def numeric(value, name, positive=True):
        if type(value) not in (int, float) or not math.isfinite(value) or (value <= 0 if positive else value < 0):
            raise ValueError(f'参数未标定或非法：{name}')

    for name in arm_commands:
        value = p['arm'][name]
        if type(value) is not int or not 2 <= value <= 7:
            raise ValueError(f'arm.{name} 必须为 2..7（v0.8，8已删除，9..15保留）')
    for section, fields in {
        'arm': ('lower_speed_rpm', 'lower_duration_s'),
        'vision': ('max_frame_age_s', 'target_missing_timeout_s', 'camera_start_timeout_s'),
        'angle_alignment': ('timeout_s', 'min_horizontal_span_px', 'slope_tolerance', 'max_wz_rad_s'),
        **{s: ('tolerance_px', 'max_vx_mps', 'max_vy_mps') for s in servo_sections},
        'ring_search': ('image_horizontal_speed_mps',),
        'xy_return': ('position_tolerance_m', 'max_vx_mps', 'max_vy_mps'),
        'execution': ('control_rate_hz', 'zero_settle_time_s', 'visual_step_timeout_s',
                      'xy_return_timeout_s', 'feedback_timeout_s', 'arm_request_timeout_s', 'lease_timeout_s'),
    }.items():
        for field in fields:
            numeric(p[section][field], f'{section}.{field}')
    if p['angle_alignment']['timeout_s'] > 5 or p['angle_alignment']['target_slope'] != 0:
        raise ValueError('角度校准上限为 5 秒，目标斜率为 0')
    for section in ('angle_alignment', *servo_sections, 'xy_return'):
        for key, value in p[section].items():
            if key.endswith('pid'):
                for term in ('kp', 'ki', 'kd'):
                    numeric(value[term], f'{section}.{key}.{term}', positive=term == 'kp')
        if section != 'xy_return' and (type(p[section]['settle_frames']) is not int or p[section]['settle_frames'] < 1):
            raise ValueError(f'{section}.settle_frames 必须为正整数')
    point = p['vision']['target_uv_px']
    if not isinstance(point, list) or len(point) not in (0, 2):
        raise ValueError('target_uv_px 须为空或 [u,v]')
    for v in point:
        numeric(v, 'target_uv_px', positive=False)
    for name in ('ring_config', 'material_config', 'model_runtime_path'):
        if not Path(p['vision'][name]).exists():
            raise ValueError(f'不存在的视觉配置/运行库：{name}')


class PlanarWorkTask:
    RETURN_REFERENCE_STATE = 'WORK_EXIT'
    MATERIAL_VERIFY_STATE = 'VERIFY_PICK'
    ARM_STATES = {'WORK_ENTRY': 'work_pose_cmd', 'LOWER': None, 'PLACE': 'place_cmd',
                  'PICK': 'pick_cmd', 'WORK_EXIT': 'work_pose_cmd', 'STOW': 'stow_cmd'}
    ANGLE_STATES = ('ALIGN_ENTRY', 'ALIGN_EXIT')
    RING_STATES = ('FIND_RING', 'ALIGN_RING', 'VERIFY_PLACE')
    MATERIAL_STATES = ('ALIGN_MATERIAL', 'VERIFY_PICK')

    def __init__(self, config, task_id, colors, positions, now):
        self.p, self.task_id = config, task_id
        self.colors, self.positions = tuple(colors), tuple(positions)
        self.item = 0
        self.reference = None
        self.placements = {}
        self.confirmed = []
        self.entry_alignment = self.final_alignment = 'NOT_RUN'
        self.returned = False
        self.reason = ''
        self.diagnostic = {}
        self.velocity = (0., 0., 0.)
        self._enter('WORK_ENTRY', now)

    @property
    def vision_kind(self):
        if self.state in (*self.ANGLE_STATES, *self.RING_STATES):
            return 'ring'
        if self.state in self.MATERIAL_STATES:
            return 'material'
        return ''

    @property
    def vision_color(self):
        return self.colors[min(self.item, 2)] if self.vision_kind == 'material' else 0

    def _servo_config(self):
        return self.p['place_servo' if self.state in self.RING_STATES else 'pick_servo']

    def _target_uv(self, obs):
        return self.p['vision']['target_uv_px'] or [(obs.image_size[0]-1)/2, (obs.image_size[1]-1)/2]

    @property
    def finished(self):
        return self.state in ('DONE', 'ERROR', 'STOPPED')

    def _enter(self, state, now):
        previous = getattr(self, 'state', '')
        if state == 'FIND_RING' or (state == 'ALIGN_MATERIAL' and previous == 'RETURN_ITEM'):
            self.visual_started = now
        self.state, self.entered_at = state, now
        self.velocity = (0., 0., 0.)
        self.arm_request = None
        self.last_sequence = None
        self.last_observed_at = None
        self.count = 0
        self.zero_since = self.missing_since = None
        self._pids = []
        if state in self.ARM_STATES:
            key = self.ARM_STATES[state]
            self.arm_request = dict(command_id=f'{self.task_id}:{state}:{self.item}',
                                    cmd=None if key is None else self.p['arm'][key])
        elif state in self.ANGLE_STATES:
            c = self.p['angle_alignment']
            self._pids = [PID(c['pid'], c['max_wz_rad_s'])]
        elif state in ('ALIGN_RING', 'ALIGN_MATERIAL'):
            c = self._servo_config()
            self._pids = [PID(c['u_pid'], c['max_vx_mps']), PID(c['v_pid'], c['max_vy_mps'])]
        elif state in ('RETURN_ITEM', 'RETURN_REFERENCE'):
            c = self.p['xy_return']
            self._pids = [PID(c['x_pid'], c['max_vx_mps']), PID(c['y_pid'], c['max_vy_mps'])]

    def stop(self, now, reason, failed=True):
        if not self.finished:
            self._enter('ERROR' if failed else 'STOPPED', now)
            self.reason = reason

    def tick(self, now, pose, observation=None, arm=None):
        if self.finished:
            return self.velocity
        if self.state in self.ARM_STATES:
            self._wait_arm(now, arm)
        elif self.state in ('CAPTURE_REFERENCE', 'RECORD_PLACE'):
            self._record(now, pose)
        elif self.state in ('RETURN_ITEM', 'RETURN_REFERENCE'):
            self._return(now, pose)
        elif self.vision_kind:
            self._vision(now, observation)
        return self.velocity

    def _wait_arm(self, now, arm):
        if arm is None or arm.command_id != self.arm_request['command_id']:
            return
        if arm.state in ('FAILED', 'UNKNOWN'):
            raise RuntimeError(arm.reason)
        if arm.state != 'COMPLETED':
            return
        if self.state == 'PICK':
            self._confirm('PICK')
            self.item += 1
            self._enter('RETURN_ITEM' if self.item < 3 else 'RETURN_REFERENCE', now)
        else:
            self._enter({'WORK_ENTRY': 'ALIGN_ENTRY', 'LOWER': 'CAPTURE_REFERENCE',
                         'PLACE': 'RECORD_PLACE', 'WORK_EXIT': 'ALIGN_EXIT', 'STOW': 'DONE'}[self.state], now)

    def _confirm(self, operation):
        self.confirmed.append((operation, self.item+1, self.colors[self.item], self.positions[self.item]))

    def _record(self, now, pose):
        if pose.received_at <= self.entered_at:
            return
        xy = (pose.x, pose.y)
        if self.state == 'CAPTURE_REFERENCE':
            self.reference = xy
            self._enter('FIND_RING', now)
        else:
            self.placements[self.positions[self.item]] = xy
            self._confirm('PLACE')
            self.item += 1
            if self.item == 3:
                self.item = 0
                self._enter('RETURN_ITEM', now)
            else:
                self._enter('FIND_RING', now)

    def _return(self, now, pose):
        if now-self.entered_at > self.p['execution']['xy_return_timeout_s']:
            raise RuntimeError('二维位置返回超时')
        c = self.p['xy_return']
        target = self.reference if self.state == 'RETURN_REFERENCE' else self.placements[self.positions[self.item]]
        ex, ey = target[0]-pose.x, target[1]-pose.y
        self.diagnostic = dict(target_xy=target, current_xy=(pose.x, pose.y), error_xy=(ex, ey))
        if math.hypot(ex, ey) <= c['position_tolerance_m']:
            self.velocity = (0., 0., 0.)
            if self.zero_since is None:
                self.zero_since = now
            elif now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and pose.received_at > self.zero_since:
                if self.state == 'RETURN_REFERENCE':
                    self.returned = True
                    self._enter(self.RETURN_REFERENCE_STATE, now)
                else:
                    self._enter('ALIGN_MATERIAL', now)
            return
        self.zero_since = None
        self.velocity = return_velocity(self._pids, pose, target, c)

    def _vision(self, now, obs):
        angle = self.state in self.ANGLE_STATES
        timeout = self.p['angle_alignment']['timeout_s'] if angle else self.p['execution']['visual_step_timeout_s']
        started = self.entered_at if angle else self.visual_started
        if now-started >= timeout:
            if angle:
                self._angle_done(now, 'TIMEOUT_SKIPPED')
                return
            raise RuntimeError('目标寻找/视觉对准超时')
        fresh = obs is not None and obs.kind == self.vision_kind and obs.captured_at > self.entered_at and 0 <= now-obs.captured_at <= self.p['vision']['max_frame_age_s']
        if not fresh:
            self._missing(now, fatal=not angle)
            return
        if obs.sequence == self.last_sequence:
            return
        if (self.last_sequence is not None and self.last_observed_at is not None
                and obs.captured_at-self.last_observed_at > self.p['vision']['max_frame_age_s']):
            self._missing(now, fatal=False)
        self.last_sequence = obs.sequence
        self.last_observed_at = obs.captured_at
        if angle:
            self._angle(now, obs)
            return
        point = self._target(obs)
        if point is None:
            self._missing(now)
            if self.state == 'FIND_RING' and obs.rings:
                self.missing_since = None
                visible = min(obs.rings, key=lambda n: abs(n-self.positions[self.item]))
                direction = 1 if self.positions[self.item] > visible else -1
                self.velocity = (-direction*self.p['ring_search']['image_horizontal_speed_mps'], 0., 0.)
            return
        self.missing_since = None
        if self.state == 'FIND_RING':
            self._enter('ALIGN_RING', now)
            return
        self._align_target(now, obs, point)

    def _align_target(self, now, obs, point):
        c = self._servo_config()
        target = self._target_uv(obs)
        eu, ev = point[0]-target[0], point[1]-target[1]
        self.diagnostic = dict(center=point, target_uv=target, error_uv=(eu, ev))
        good = math.hypot(eu, ev) <= c['tolerance_px']
        if self.state in ('VERIFY_PLACE', 'VERIFY_PICK', 'VERIFY_STACK'):
            if not good:
                self._enter('ALIGN_RING' if self.state == 'VERIFY_PLACE' else 'ALIGN_MATERIAL', now)
            elif now-self.entered_at >= self.p['execution']['zero_settle_time_s']:
                self._enter({'VERIFY_PLACE': 'PLACE', 'VERIFY_PICK': 'PICK', 'VERIFY_STACK': 'STACK'}[self.state], now)
            return
        if good:
            self.velocity = (0., 0., 0.)
            self.count += 1
            if self.count >= c['settle_frames']:
                self._enter('VERIFY_PLACE' if self.state == 'ALIGN_RING' else self.MATERIAL_VERIFY_STATE, now)
        else:
            self.count = 0
            self.velocity = (-self._pids[0].update(eu, obs.captured_at), self._pids[1].update(ev, obs.captured_at), 0.)

    def _target(self, obs):
        if self.state in self.RING_STATES:
            return obs.rings.get(self.positions[self.item])
        target = self._target_uv(obs)
        points = [point for color, point in obs.materials if color == self.colors[self.item]]
        return min(points, key=lambda xy: math.hypot(xy[0]-target[0], xy[1]-target[1])) if points else None

    def _missing(self, now, fatal=True):
        self.velocity = (0., 0., 0.)
        self.count = 0
        self.zero_since = None
        for pid in self._pids:
            pid.reset()
        if self.missing_since is None:
            self.missing_since = now
        if fatal and now-self.missing_since > self.p['vision']['target_missing_timeout_s']:
            raise RuntimeError('目标持续缺失或图像过期')

    def _angle(self, now, obs):
        c = self.p['angle_alignment']
        slope = line_slope(list(obs.rings.values()), c['min_horizontal_span_px'])
        self.diagnostic = dict(slope=slope, centers=obs.rings)
        if slope is None:
            self._missing(now, fatal=False)
            return
        if abs(slope) <= c['slope_tolerance']:
            self.velocity = (0., 0., 0.)
            if self.zero_since is not None:
                if now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and obs.captured_at > self.zero_since:
                    self._angle_done(now, 'ALIGNED')
            else:
                self.count += 1
                if self.count >= c['settle_frames']:
                    self.zero_since = now
        else:
            self.zero_since = None
            self.count = 0
            self.velocity = (0., 0., self._pids[0].update(-slope, obs.captured_at))

    def _angle_done(self, now, outcome):
        if self.state == 'ALIGN_ENTRY':
            self.entry_alignment = outcome
            self._enter('LOWER', now)
        else:
            self.final_alignment = outcome
            self._enter('STOW', now)
