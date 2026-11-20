"""Coarse processing: calibrate the base once, place visually, retrieve by gantry feedback."""

import math
from pathlib import Path


from .work_config import read_work_config
from .work_control import GantryPose, Observation, PID, Pose, line_slope, return_velocity
from .gantry_presets import PresetMove, validate_presets


def read_coarse_config(path):
    return read_work_config(path)['coarse']


def validate_gantry_work_config(p, commands, with_lower=True):
    validate_presets(p.get('gantry_presets'))
    def number(value, name, positive=True):
        if type(value) not in (int, float) or not math.isfinite(value) or (value <= 0 if positive else value < 0):
            raise ValueError(f'参数未标定或非法：{name}')

    for name in commands:
        if type(p['arm'][name]) is not int or not 2 <= p['arm'][name] <= 7:
            raise ValueError(f'arm.{name} 必须为 2..7（v0.8，8已删除，9..15保留）')
    numeric = {
        'arm': ('lower_speed_rpm', 'lower_duration_s'),
        'vision': ('max_frame_age_s', 'camera_start_timeout_s'),
        'angle_alignment': ('timeout_s', 'min_horizontal_span_px', 'slope_tolerance', 'max_wz_rad_s'),
        'base_servo': ('tolerance_px', 'timeout_s', 'max_vx_mps', 'max_vy_mps', 'stop_speed_mps'),
        'gantry_servo': ('tolerance_px', 'timeout_s', 'max_yaw_rad_s', 'max_fb_rpm', 'small_yaw_rad_s', 'small_fb_rpm'),
        'ring_search': ('yaw_rad_s', 'duration_s'),
        'gantry_motion': ('length_tolerance_pulses', 'angle_tolerance_rad', 'max_yaw_rad_s', 'max_fb_rpm', 'timeout_s'),
        'execution': ('control_rate_hz', 'zero_settle_time_s', 'feedback_timeout_s', 'arm_request_timeout_s', 'lease_timeout_s'),
    }
    if not with_lower:
        del numeric['arm']
    for section, keys in numeric.items():
        for key in keys:
            number(p[section][key], f'{section}.{key}')
    if p['angle_alignment']['timeout_s'] > 3 or p['angle_alignment']['target_slope'] != 0:
        raise ValueError('角度校准上限为 3 秒，目标斜率为 0')
    if p['gantry_servo']['qualified_frames'] != 4:
        raise ValueError('机构放料须连续4张新帧达标')
    if p['execution']['control_rate_hz'] != 50:
        raise ValueError('机构控制频率须为50 Hz')
    for section in ('angle_alignment', 'base_servo', 'gantry_servo', 'gantry_motion'):
        for key, value in p[section].items():
            if key.endswith('pid'):
                for term in ('kp', 'ki', 'kd'):
                    number(value[term], f'{section}.{key}.{term}', term == 'kp')
        if section in ('angle_alignment', 'base_servo'):
            n = p[section]['settle_frames']
            if type(n) is not int or n < 1:
                raise ValueError(f'{section}.settle_frames 必须为正整数')
    point = p['vision']['target_uv_px']
    if not isinstance(point, list) or len(point) not in (0, 2):
        raise ValueError('target_uv_px 须为空或 [u,v]')
    for v in point:
        number(v, 'target_uv_px', False)
    for name in ('ring_config', 'material_config', 'model_runtime_path'):
        if not Path(p['vision'][name]).exists():
            raise ValueError(f'不存在的视觉配置/运行库：{name}')


def validate_coarse_config(p):
    validate_gantry_work_config(p, ('work_pose_cmd', 'place_cmd', 'pick_cmd', 'stow_cmd'),
                               with_lower=False)
    validate_exit_motion_config(p)


def validate_exit_motion_config(p):
    for key in ('max_vx_mps', 'max_vy_mps', 'position_tolerance_m', 'kp', 'timeout_s'):
        value = p['exit_motion'][key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'参数非法：exit_motion.{key}')


class CoarseTask:
    CENTER_NEXT_STATE = 'ALIGN_RING'
    GANTRY_NEXT_STATE = 'STOP_PLACE'
    ARM_STATES = {'WORK_ENTRY': 'work_pose_cmd', 'PLACE': 'place_cmd',
                  'PICK': 'pick_cmd', 'WORK_EXIT': 'work_pose_cmd', 'STOW': 'stow_cmd'}
    ANGLE_STATES = ('ALIGN_ENTRY', 'ALIGN_EXIT')
    RING_STATES = ('CENTER_RING2', 'ALIGN_RING')

    def __init__(self, config, task_id, colors, positions, now, *, defer_reference_return=False):
        self.p, self.task_id = config, task_id
        self.colors, self.positions = tuple(colors), tuple(positions)
        self.item = 0
        self.reference = None  # P0 after entry angle correction, before ring-2 translation.
        self.placements = {}  # ring_id -> (actual FB pulses, gantry yaw rad)
        self.confirmed = []
        self.entry_alignment = self.final_alignment = 'NOT_RUN'
        self.returned = False
        self.defer_reference_return = defer_reference_return
        self.reference_return_deferred = False
        self.reference_return_timed_out = False
        self.return_timed_out = False
        self.reason = ''
        self.diagnostic = {}
        self.work_gantry = None
        self.preset_move = None
        self._enter('WORK_ENTRY', now)

    @property
    def vision_kind(self):
        if self.state in ('PREPOSITION', 'STOP_PREPOSITION'):
            return 'material' if self.preset_next == 'ALIGN_MATERIAL' else 'ring'
        return 'ring' if self.state in (*self.ANGLE_STATES, *self.RING_STATES) else ''

    @property
    def vision_color(self):
        return 0

    @property
    def vision_target(self):
        return self.positions[self.item] if self.state == 'ALIGN_RING' else 0

    @property
    def finished(self):
        return self.state in ('DONE', 'ERROR', 'STOPPED')

    def _enter(self, state, now):
        if state in ('ALIGN_RING', 'ALIGN_MATERIAL') and getattr(self, 'state', '') != 'STOP_PREPOSITION':
            target = self._preset_target()
            if target is not None:
                self.preset_next = state
                self.preset_move = PresetMove(self.p['gantry_motion'], target, now,
                                              self.p['execution']['feedback_timeout_s'])
                state = 'PREPOSITION'
        self.state, self.entered_at = state, now
        self.velocity, self.arm_velocity = (0., 0., 0.), (0., 0.)
        self.servo_enabled = False
        self.arm_request = None
        self.last_sequence = self.last_observed_at = None
        self.count = 0
        self.zero_since = self.search_started = None
        self._alignment_started = None
        self._pids = []
        if state in self.ARM_STATES:
            key = self.ARM_STATES[state]
            self.arm_request = dict(command_id=f'{self.task_id}:{state}:{self.item}',
                                    cmd=None if key is None else self.p['arm'][key])
        elif state in self.ANGLE_STATES:
            c = self.p['angle_alignment']
            self._pids = [PID(c['pid'], c['max_wz_rad_s'])]
        elif state == 'CENTER_RING2':
            c = self.p['base_servo']
            self._pids = [PID(c['u_pid'], c['max_vx_mps']), PID(c['v_pid'], c['max_vy_mps'])]
        elif state in ('ALIGN_RING', 'RETURN_ITEM'):
            c = self.p['gantry_servo' if state == 'ALIGN_RING' else 'gantry_motion']
            keys = ('u_pid', 'v_pid') if state == 'ALIGN_RING' else ('yaw_pid', 'length_pid')
            self._pids = [PID(c[keys[0]], c['max_yaw_rad_s']), PID(c[keys[1]], c['max_fb_rpm'])]
        elif state == 'RETURN_REFERENCE':
            c = self.p['exit_motion']
            gains = dict(kp=c['kp'], ki=0., kd=0.)
            self._pids = [PID(gains, max(c['max_vx_mps'], c['max_vy_mps'])) for _ in range(2)]

    def stop(self, now, reason, failed=True):
        if not self.finished:
            self._enter('ERROR' if failed else 'STOPPED', now)
            self.reason = reason

    def tick(self, now, pose, observation=None, arm=None, gantry=None):
        if self.finished:
            return self.velocity
        if self.state in ('STOP_PLACE', 'STOP_PICK', 'RETURN_ITEM') and gantry is None:
            raise RuntimeError('机构前后位置反馈无效，无法记录或返回脉冲位置')
        if ((self.state == 'CENTER_RING2' or self.work_gantry is None and self.state == 'SCAN_FIRST_LAYER') and
                gantry and gantry.received_at > self.entered_at):
            self.work_gantry = dict(fb_pulses=gantry.fb_pulses, yaw_rad=gantry.yaw_rad)
        # Expired PID transitions cannot depend on another target observation.
        if self.state in ('CENTER_RING2', 'ALIGN_RING', 'ALIGN_MATERIAL') and self._alignment_started is not None:
            c = self.p['base_servo' if self.state == 'CENTER_RING2' else 'gantry_servo']
            if self._alignment_expired(now, c):
                self.velocity, self.arm_velocity = (0., 0., 0.), (0., 0.)
                self.servo_enabled = False
                if self.state == 'CENTER_RING2':
                    self._center(now, pose, None, 0., 0.)
                else:
                    self._enter(self.GANTRY_NEXT_STATE, now)
                return self.velocity
        if self.state in self.ARM_STATES:
            self._wait_arm(now, arm)
        elif self.state in ('PREPOSITION', 'STOP_PREPOSITION'):
            self._preposition(now, gantry, arm)
        elif self.state == 'STOP_PLACE':
            if (gantry and gantry.received_at > self.entered_at and arm and arm.idle and not arm.pending and
                    now-self.entered_at >= self.p['execution']['zero_settle_time_s']):
                self.placements[self.positions[self.item]] = (gantry.fb_pulses, gantry.yaw_rad)
                self._enter('PLACE', now)
        elif self.state == 'STOP_PICK':
            if now-self.return_started >= self.p['gantry_motion']['timeout_s']:
                self.return_timed_out = True
                self.reason = '机构长度/角度PID返回超时，停止纠偏继续取回；位置未确认达标'
            if (gantry and gantry.received_at > self.entered_at and arm and arm.idle and not arm.pending and
                    now-self.entered_at >= self.p['execution']['zero_settle_time_s']):
                # Recheck the actual axes after stopping before triggering the fixed pick.
                if self.return_timed_out or self._return_good(gantry):
                    self._enter('PICK', now)
                else:
                    started = self.return_started
                    self._enter('RETURN_ITEM', now)
                    self.return_started = started
        elif self.state == 'RETURN_ITEM':
            self._return(now, gantry)
        elif self.state == 'CAPTURE_REFERENCE':
            self._capture_reference(now, pose)
        elif self.state == 'RETURN_REFERENCE':
            self._return_reference(now, pose)
        elif self.state == 'STOP_EXIT':
            self._stop_exit(now, pose)
        elif self.vision_kind:
            self._vision(now, pose, observation)
        return self.velocity

    def _preset_target(self):
        config = self.p.get('gantry_presets')
        if not config or not any(config['positions'].values()):
            return None
        ring = self.positions[self.item]
        return self.work_gantry if ring == 2 else config['positions'][str(ring)]

    def _preposition(self, now, gantry, arm):
        good, velocity, self.diagnostic = self.preset_move.update(now, gantry)
        self.diagnostic['target_ring'] = self.positions[self.item]
        if self.preset_move.timed_out:
            self.reason = '机构标定点预定位PID超时，停止纠偏继续视觉对准；位置未确认达标'
        if self.state == 'PREPOSITION':
            if good:
                self._enter('STOP_PREPOSITION', now)
            else:
                self.arm_velocity, self.servo_enabled = velocity, True
        elif not good:
            self._enter('PREPOSITION', now)
        elif (gantry.received_at > self.entered_at and arm and arm.idle and not arm.pending and
              now-self.entered_at >= self.p['execution']['zero_settle_time_s']):
            self._enter(self.preset_next, now)

    def _wait_arm(self, now, arm):
        if arm is None or arm.command_id != self.arm_request['command_id']:
            return
        if arm.state in ('FAILED', 'UNKNOWN'):
            raise RuntimeError(arm.reason)
        if arm.state != 'COMPLETED':
            return
        self._arm_completed(now)

    def _arm_completed(self, now):
        if self.state in ('PLACE', 'PICK'):
            op = self.state
            self.confirmed.append((op, self.item+1, self.colors[self.item], self.positions[self.item]))
            self.item += 1
            if op == 'PLACE':
                if self.item == 3:
                    self.item = 0
                    self.return_started = now
                    self.return_timed_out = False
                    self._enter('RETURN_ITEM', now)
                else:
                    self._enter('ALIGN_RING', now)
            elif self.item < 3:
                self.return_started = now
                self.return_timed_out = False
                self._enter('RETURN_ITEM', now)
            else:
                self._enter('WORK_EXIT', now)
        else:
            self._enter({'WORK_ENTRY': 'ALIGN_ENTRY',
                         'WORK_EXIT': 'ALIGN_EXIT',
                         'STOW': 'STOP_EXIT' if self.defer_reference_return else 'RETURN_REFERENCE'}[self.state], now)

    def _stop_exit(self, now, pose):
        if now-self.entered_at > self.p['exit_motion']['timeout_s']:
            raise RuntimeError('收纳后原地停车交接超时')
        if pose.received_at <= self.entered_at:
            return
        if math.hypot(pose.vx, pose.vy) > self.p['base_servo']['stop_speed_mps']:
            self.zero_since = None
        elif self.zero_since is None:
            self.zero_since = now
        elif (now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and
              pose.received_at > self.zero_since):
            self.reference_return_deferred = self.defer_reference_return and not self.reference_return_timed_out
            self._enter('DONE', now)

    def _capture_reference(self, now, pose):
        if pose.received_at <= self.entered_at:
            return
        if math.hypot(pose.vx, pose.vy) > self.p['base_servo']['stop_speed_mps']:
            self.zero_since = None
        elif self.zero_since is None:
            self.zero_since = now
        elif (now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and
              pose.received_at > self.zero_since):
            self.reference = (pose.x, pose.y)
            self._enter('CENTER_RING2', now)

    def _return_reference(self, now, pose):
        c = self.p['exit_motion']
        if now-self.entered_at >= c['timeout_s']:
            self.reference_return_timed_out = True
            self.reason = '底盘返回P0的PID超时，停止纠偏继续流程；P0未确认恢复'
            self._enter('STOP_EXIT', now)
            return
        if pose.received_at <= self.entered_at:
            return
        ex, ey = self.reference[0]-pose.x, self.reference[1]-pose.y
        self.diagnostic = dict(target_xy=self.reference, current_xy=(pose.x, pose.y), error_xy=(ex, ey))
        if math.hypot(ex, ey) > c['position_tolerance_m']:
            self.zero_since = None
            self.velocity = return_velocity(self._pids, pose, self.reference, c)
            return
        self.velocity = (0., 0., 0.)
        if self.zero_since is None:
            self.zero_since = now
        elif (now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and
              pose.received_at > self.zero_since and
              math.hypot(pose.vx, pose.vy) <= self.p['base_servo']['stop_speed_mps']):
            self.returned = True
            self._enter('DONE', now)

    def _vision(self, now, pose, obs):
        angle = self.state in self.ANGLE_STATES
        if angle and now-self.entered_at >= self.p['angle_alignment']['timeout_s']:
            self._angle_done(now, 'TIMEOUT_SKIPPED')
            return
        fresh = (obs is not None and obs.kind == 'ring' and obs.captured_at > self.entered_at and
                 obs.geometry_target in (0, self.vision_target) and
                 0 <= now-obs.captured_at <= self.p['vision']['max_frame_age_s'])
        if not fresh:
            self._missing()
            return
        if obs.sequence == self.last_sequence:
            return
        if self.last_observed_at is not None and obs.captured_at-self.last_observed_at > self.p['vision']['max_frame_age_s']:
            self._missing()
        self.last_sequence, self.last_observed_at = obs.sequence, obs.captured_at
        if angle:
            self._angle(now, obs)
            return
        ring = 2 if self.state == 'CENTER_RING2' else self.positions[self.item]
        point = obs.rings.get(ring)
        if point is None:
            self._missing()
            self.diagnostic = dict(target_ring=ring, wait_reason='等待目标圆环')
            if self.state == 'ALIGN_RING' and ring in (1, 3):
                c = self.p['ring_search']
                if self.search_started is None:
                    self.search_started = now
                if now-self.search_started < c['duration_s']:
                    self.arm_velocity = ((1 if ring == 1 else -1)*c['yaw_rad_s'], 0.)
                    self.servo_enabled = True
            return
        target = self.p['vision']['target_uv_px'] or [(obs.image_size[0]-1)/2, (obs.image_size[1]-1)/2]
        eu, ev = point[0]-target[0], point[1]-target[1]
        self.diagnostic = dict(target_ring=ring, center=point, target_uv=target, error_uv=(eu, ev))
        if self.state == 'CENTER_RING2':
            self._center(now, pose, obs, eu, ev)
        else:
            self._align_ring(now, obs, eu, ev)

    def _center(self, now, pose, obs, eu, ev):
        c = self.p['base_servo']
        expired = self._alignment_expired(now, c)
        if math.hypot(eu, ev) <= c['tolerance_px'] or expired:
            self.velocity = (0., 0., 0.)
            self.count = ((c['settle_frames'] if expired else self.count+1)
                          if math.hypot(pose.vx, pose.vy) <= c['stop_speed_mps'] else 0)
            if self.count >= c['settle_frames']:
                if self.zero_since is None:
                    self.zero_since = now
                elif (now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and
                      (expired or obs.captured_at > self.zero_since) and pose.received_at > self.zero_since):
                    self._enter(self.CENTER_NEXT_STATE, now)
        else:
            self.count, self.zero_since = 0, None
            self.velocity = (-self._pids[0].update(eu, obs.captured_at), self._pids[1].update(ev, obs.captured_at), 0.)

    def _align_ring(self, now, obs, eu, ev):
        c = self.p['gantry_servo']
        if self._alignment_expired(now, c):
            self._enter(self.GANTRY_NEXT_STATE, now)
            return
        yaw = -self._pids[0].update(eu, obs.captured_at)
        fb = -self._pids[1].update(ev, obs.captured_at)
        self.arm_velocity, self.servo_enabled = (yaw, fb), True
        good = (math.hypot(eu, ev) <= c['tolerance_px'] and
                abs(yaw) <= c['small_yaw_rad_s'] and abs(fb) <= c['small_fb_rpm'])
        self.count = self.count+1 if good else 0
        self.diagnostic.update(yaw_wz=yaw, fb_rpm=fb, qualified_frames=self.count)
        if self.count >= c['qualified_frames']:
            self._enter(self.GANTRY_NEXT_STATE, now)

    def _alignment_expired(self, now, config):
        if self._alignment_started is None:
            self._alignment_started = now
        elapsed = now-self._alignment_started
        self.diagnostic.update(alignment_elapsed_s=elapsed, alignment_limit_s=config['timeout_s'])
        if elapsed < config['timeout_s']:
            return False
        kind = '物料' if self.state == 'ALIGN_MATERIAL' else '圆环'
        self.reason = f'{self.state} {kind}PID达到{config["timeout_s"]:g}秒限时，停止纠偏继续流程；视觉对准未确认达标'
        self.diagnostic.update(alignment_outcome='TIMEOUT_SKIPPED', wait_reason=self.reason)
        return True

    def _return_good(self, gantry):
        c = self.p['gantry_motion']
        length, yaw = self.placements[self.positions[self.item]]
        return abs(length-gantry.fb_pulses) <= c['length_tolerance_pulses'] and abs(yaw-gantry.yaw_rad) <= c['angle_tolerance_rad']

    def _return(self, now, gantry):
        if gantry is None:
            raise RuntimeError('缺少机构实际位置反馈')
        length, yaw = self.placements[self.positions[self.item]]
        el, et = length-gantry.fb_pulses, yaw-gantry.yaw_rad
        self.diagnostic = dict(target_ring=self.positions[self.item], target_rt=(length, yaw),
                               current_rt=(gantry.fb_pulses, gantry.yaw_rad), error_rt=(el, et))
        if now-self.return_started >= self.p['gantry_motion']['timeout_s']:
            self.return_timed_out = True
            self.reason = '机构长度/角度PID返回超时，停止纠偏继续取回；位置未确认达标'
            self.diagnostic['motion_outcome'] = 'TIMEOUT_SKIPPED'
        if self.return_timed_out or self._return_good(gantry):
            self._enter('STOP_PICK', now)
        else:
            # The mechanism has limited angular travel; use the recorded absolute angle.
            self.arm_velocity = (self._pids[0].update(et, gantry.received_at),
                                 self._pids[1].update(el, gantry.received_at))
            self.servo_enabled = True

    def _missing(self):
        self.velocity, self.arm_velocity = (0., 0., 0.), (0., 0.)
        self.servo_enabled = False
        self.count, self.zero_since = 0, None
        for pid in self._pids:
            pid.reset()

    def _angle(self, now, obs):
        c = self.p['angle_alignment']
        slope = line_slope(list(obs.rings.values()), c['min_horizontal_span_px'])
        self.diagnostic = dict(slope=slope, centers=obs.rings)
        if slope is None:
            self._missing()
        elif abs(slope) <= c['slope_tolerance']:
            self.velocity = (0., 0., 0.)
            if self.zero_since is not None:
                if now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and obs.captured_at > self.zero_since:
                    self._angle_done(now, 'ALIGNED')
            else:
                self.count += 1
                if self.count >= c['settle_frames']:
                    self.zero_since = now
        else:
            self.zero_since, self.count = None, 0
            self.velocity = (0., 0., self._pids[0].update(-slope, obs.captured_at))

    def _angle_done(self, now, outcome):
        if self.state == 'ALIGN_ENTRY':
            self.entry_alignment = outcome
            self._enter('CAPTURE_REFERENCE', now)
        else:
            self.final_alignment = outcome
            self._enter('STOW', now)
