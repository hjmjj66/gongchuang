"""Single-target visual tracking without chassis, lift or automatic arm programs."""

import math

from agv_perception.work_control import PID, target_uv
from agv_perception.raw_servo import RawGantryServo


def parse_tracking_target(settings, kind):
    target = settings.get('target')
    maximum = 6 if kind == 'material' else 3
    if type(target) is not int or not 1 <= target <= maximum:
        raise ValueError(f'请选择有效的{"物料颜色" if kind == "material" else "圆环编号"}（1～{maximum}）')
    return target


class TrackingTask:
    def __init__(self, config, task_id, target, kind, now, area, visit=1, *, fb_only=False):
        self.p, self.task_id, self.target, self.kind = config, task_id, target, kind
        self.fb_only = fb_only
        self.area = area
        self.stack = area == 'storage' and visit == 2
        self.action_key = ('pick_cmd' if area == 'raw' else
                           'stack_cmd' if area == 'storage' and visit == 2 else 'place_cmd')
        self.state, self.entered_at = 'TRACK_WAIT', now
        self.reason, self.diagnostic = '', {}
        self.velocity, self.arm_velocity = (0., 0., 0.), (0., 0.)
        self.servo_enabled, self.arm_request = False, None
        self.last_sequence = self.last_observed_at = None
        self.item = 0
        self.colors = self.positions = (target,)
        self.reference, self.placements, self.confirmed = None, {}, []
        self.entry_alignment = self.final_alignment = 'NOT_RUN'
        c = config['gantry_servo']
        self._raw_servo = RawGantryServo(config) if area == 'raw' else None
        self._pids = (self._raw_servo.pids if self._raw_servo else
                      [PID(c['u_pid'], c['max_yaw_rad_s']), PID(c['v_pid'], c['max_fb_rpm'])])

    @property
    def finished(self):
        return self.state in ('DONE', 'ERROR', 'STOPPED')

    @property
    def vision_target(self):
        return self.target

    def _zero(self):
        self.arm_velocity, self.servo_enabled = (0., 0.), False
        if self._raw_servo:
            self._raw_servo.reset()
        else:
            for pid in self._pids:
                pid.reset()

    def stop(self, now, reason, failed=True):
        if not self.finished:
            self._zero()
            self.arm_request = None
            self.state, self.reason = ('ERROR' if failed else 'STOPPED'), reason

    def request_action(self, now):
        if self.finished or self.state in ('TRACK_ACTION_STOP', 'TRACK_ACTION'):
            raise ValueError('请先开始追踪；每次追踪只执行一次取放动作')
        self._zero()
        self.state, self.action_at = 'TRACK_ACTION_STOP', now
        self.diagnostic = dict(wait_reason='停止追踪，等待机构停稳后执行真实取放')

    def tick(self, now, pose=None, observation=None, arm=None, gantry=None, arm_at=None):
        if self.finished:
            return self.velocity
        if self.state == 'TRACK_ACTION_STOP':
            if now-self.action_at > self.p['execution']['arm_request_timeout_s']:
                raise RuntimeError('取放前机构停稳等待超时')
            if (arm and arm.idle and not arm.pending and arm_at is not None and
                    arm_at > self.action_at and
                    now-self.action_at >= self.p['execution'].get('zero_settle_time_s', .1)):
                self.arm_request = dict(command_id=f'{self.task_id}:manual', cmd=self.p['arm'][self.action_key])
                self.state = 'TRACK_ACTION'
            return self.velocity
        if self.state == 'TRACK_ACTION':
            if arm and arm.command_id == self.arm_request['command_id']:
                if arm.state in ('FAILED', 'UNKNOWN'):
                    raise RuntimeError(arm.reason or '取放动作失败')
                if arm.state == 'COMPLETED':
                    self.confirmed.append(('MANUAL', 1, self.target, self.target))
                    self.state, self.arm_request = 'DONE', None
            return self.velocity
        obs = observation
        limit = self.p['vision']['max_frame_age_s']
        if (obs is None or obs.kind != self.kind or
                not 0 <= now-obs.captured_at <= limit):
            self._zero()
            raise RuntimeError('主相机识别图像过期/断流')
        if obs.captured_at <= self.entered_at:
            self._zero()
            self.state = 'TRACK_WAIT'
            if now-self.entered_at > self.p['vision']['camera_start_timeout_s']:
                raise RuntimeError('等待追踪开始后的新图像超时')
            return self.velocity
        if obs.geometry_target not in (0, self.vision_target):
            self._zero()
            return self.velocity
        if obs.sequence == self.last_sequence:
            if self._raw_servo and self._raw_servo.active and self.p['motion_prediction']['enabled']:
                self._update_raw_servo(now)
            return self.velocity
        if self.last_observed_at is not None and obs.captured_at <= self.last_observed_at:
            self._zero()
            raise RuntimeError('追踪图像时间未递增')
        self.last_sequence, self.last_observed_at = obs.sequence, obs.captured_at
        point = (obs.rings.get(self.target) if self.kind == 'ring' else
                 next((p for color, p in obs.materials if color == self.target), None))
        if point is None:
            self._zero()
            self.state = 'TRACK_WAIT'
            self.diagnostic = dict(target=self.target, wait_reason='目标不可见，停轴等待同一目标')
            return self.velocity
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            self._zero()
            raise ValueError('目标中心坐标无效')
        aim = target_uv(self.p, obs.image_size, stack=self.stack)
        eu, ev = point[0]-aim[0], point[1]-aim[1]
        if self._raw_servo:
            self._raw_servo.observe((eu, ev), obs.captured_at)
            self.diagnostic = dict(target=self.target, center=point, target_uv=aim)
            self._update_raw_servo(now)
            return self.velocity
        yaw, fb = (-pid.update(error, obs.captured_at)
                   for pid, error in zip(self._pids, (0. if self.fb_only else eu, ev)))
        gate = self.p['gantry_servo']
        aligned = ((abs(ev) if self.fb_only else math.hypot(eu, ev)) <= gate['tolerance_px'] and
                   abs(yaw) <= gate['small_yaw_rad_s'] and abs(fb) <= gate['small_fb_rpm'])
        self.state = 'TRACK_HOLD' if aligned else 'TRACKING'
        self.arm_velocity, self.servo_enabled = (yaw, fb), True
        self.diagnostic = dict(target=self.target, center=point, target_uv=aim,
                               error_uv=(eu, ev), yaw_wz=yaw, fb_rpm=fb, aligned=aligned)
        return self.velocity

    def _update_raw_servo(self, now):
        self.arm_velocity = self._raw_servo.update(now)
        self.servo_enabled = self._raw_servo.active
        self.state = ('TRACK_WAIT' if not self.servo_enabled else
                      'TRACK_HOLD' if self._raw_servo.aligned else 'TRACKING')
        self.diagnostic.update(self._raw_servo.diagnostic, aligned=self._raw_servo.aligned)
