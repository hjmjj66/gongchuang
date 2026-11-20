"""Raw picking after a base approach, with in-place waiting and navigation handoff."""

import math
from pathlib import Path

from .work_control import PID, return_velocity
from .raw_servo import RawGantryServo
from .work_config import read_work_config


def read_raw_config(path):
    return read_work_config(path)['raw']


def validate_raw_config(p):
    for name in ('work_pose_cmd', 'pick_cmd', 'stow_cmd'):
        if type(p['arm'][name]) is not int or not 2 <= p['arm'][name] <= 7:
            raise ValueError(f'arm.{name} 必须为2..7（v0.8，8已删除，9..15保留）')
    for section in ('base_motion', 'gantry_servo', 'grasp_gate', 'execution'):
        for name, value in p[section].items():
            terms = value.items() if isinstance(value, dict) else [(None, value)]
            for term, number in terms:
                allow_zero = term in ('ki', 'kd')
                if (type(number) not in (int, float) or not math.isfinite(number) or
                        (number < 0 if allow_zero else number <= 0)):
                    raise ValueError(f'参数未标定或非法：{section}.{name}.{term or ""}')
    for name in ('max_frame_age_s', 'camera_start_timeout_s'):
        value = p['vision'][name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'vision.{name} 非法')
    if type(p['grasp_gate']['qualified_frames']) is not int or p['grasp_gate']['qualified_frames'] < 1:
        raise ValueError('抓取连续达标帧数须为正整数')
    if p['base_motion']['position_tolerance_m'] >= p['base_motion']['distance_mm']/1000.:
        raise ValueError('底盘到位容差须小于前进距离')
    point = p['vision']['target_uv_px']
    if (not isinstance(point, list) or len(point) not in (0, 2) or
            any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in point)):
        raise ValueError('target_uv_px 须为空或非负[u,v]')
    if not Path(p['vision']['material_config']).is_file():
        raise ValueError('不存在的物料配置')
    prediction = p['motion_prediction']
    if type(prediction['enabled']) is not bool:
        raise ValueError('motion_prediction.enabled 须为布尔值')
    for name in ('measurement_std_px', 'acceleration_std_px_s2', 'initial_velocity_std_px_s',
                 'extra_delay_s', 'max_horizon_s'):
        value = prediction[name]
        if (type(value) not in (int, float) or not math.isfinite(value) or
                (value < 0 if name == 'extra_delay_s' else value <= 0)):
            raise ValueError(f'motion_prediction.{name} 非法')
    if not prediction['extra_delay_s'] <= prediction['max_horizon_s'] <= p['vision']['max_frame_age_s']:
        raise ValueError('预测时限须不小于额外延迟，且不超过图像有效期')


class RawTask:
    ARM_STATES = {'ENTER_WORK_POSE': 'work_pose_cmd', 'RESTORE_WORK': 'work_pose_cmd',
                  'PICK': 'pick_cmd', 'STOW_ARM': 'stow_cmd'}
    TRACK_STATES = ('WAIT_REQUIRED', 'TRACK_REQUIRED')

    def __init__(self, config, task_id, colors, now, *, defer_reference_return=False):
        self.p, self.task_id, self.colors = config, task_id, tuple(colors)
        self.positions = (0, 0, 0)
        self.item, self.confirmed = 0, []
        self.reference, self.returned = None, False
        self.reference_return_timed_out = False
        self.final_alignment, self.reason = 'NOT_RUN', ''
        self.diagnostic = {}
        self.defer_reference_return = defer_reference_return
        self.reference_return_deferred = False
        self.base_locked = False
        self.zero_since = None
        self._pids = []
        self.tracked_color = None
        self.last_sequence = self.last_observed_at = None
        self._gantry = RawGantryServo(config)
        self.visual_started = now
        self.count = 0
        self._action_serial = 0
        self._enter('ENTER_WORK_POSE', now)

    @property
    def vision_kind(self):
        return 'material' if self.state in self.TRACK_STATES else ''

    @property
    def vision_color(self):
        return 0

    @property
    def vision_target(self):
        return 0 if self.item >= 3 else self.colors[self.item]

    @property
    def finished(self):
        return self.state in ('DONE', 'ERROR', 'STOPPED')

    def _enter(self, state, now):
        self._gantry.reset()
        self.state, self.entered_at = state, now
        self.velocity, self.arm_velocity = (0., 0., 0.), (0., 0.)
        self.servo_enabled = False
        self.arm_request = None
        self.count = 0
        if state in self.ARM_STATES:
            self._action_serial += 1
            self.arm_request = dict(command_id=f'{self.task_id}:{state}:{self.item}:{self._action_serial}',
                                    cmd=self.p['arm'][self.ARM_STATES[state]])
        self.zero_since = None
        if state in ('APPROACH', 'RETURN_REFERENCE'):
            c = self.p['base_motion']
            gains = dict(kp=c['kp'], ki=0., kd=0.)
            self._pids = [PID(gains, max(c['max_vx_mps'], c['max_vy_mps'])) for _ in range(2)]

    def stop(self, now, reason, failed=True):
        if not self.finished:
            self._enter('ERROR' if failed else 'STOPPED', now)
            self.reason = reason

    def tick(self, now, pose, observation=None, arm=None, gantry=None):
        if not self.finished:
            if self.state in self.ARM_STATES:
                self._wait_arm(now, arm)
            elif self.state in ('APPROACH', 'RETURN_REFERENCE', 'STOP_EXIT'):
                self._base_motion(now, pose)
            else:
                self._vision(now, observation, gantry, arm)
        return self.velocity

    def _base_motion(self, now, pose):
        c = self.p['base_motion']
        if now-self.entered_at >= c['timeout_s']:
            raise RuntimeError('原料区底盘进退或停车交接超时')
        if pose is None or not 0 <= now-pose.received_at <= self.p['execution']['feedback_timeout_s']:
            raise RuntimeError('原料区底盘进退缺少新鲜里程反馈')
        if pose.received_at <= self.entered_at:
            return
        speed = math.hypot(pose.vx, pose.vy)
        if self.reference is None:
            if speed > c['stop_speed_mps']:
                self.zero_since = None
                return
            if self.zero_since is None:
                self.zero_since = now
                return
            if now-self.zero_since < self.p['execution']['zero_settle_time_s']:
                return
            self.reference = (pose.x, pose.y)
            distance = c['distance_mm']/1000.
            self.approach_target = (pose.x+distance*math.cos(pose.yaw),
                                    pose.y+distance*math.sin(pose.yaw))
            self.zero_since = None
        target = self.approach_target if self.state == 'APPROACH' else self.reference
        error = math.hypot(target[0]-pose.x, target[1]-pose.y)
        self.diagnostic = dict(target_xy=target, current_xy=(pose.x, pose.y), error_m=error)
        if self.state != 'STOP_EXIT' and error > c['position_tolerance_m']:
            self.zero_since = None
            self.velocity = return_velocity(self._pids, pose, target, c)
            return
        self.velocity = (0., 0., 0.)
        if speed > c['stop_speed_mps']:
            self.zero_since = None
        elif self.zero_since is None:
            self.zero_since = now
        elif now-self.zero_since >= self.p['execution']['zero_settle_time_s'] and pose.received_at > self.zero_since:
            if self.state == 'APPROACH':
                self.base_locked = True
                self.visual_started = now
                self._enter('WAIT_REQUIRED', now)
            else:
                self.returned = self.state == 'RETURN_REFERENCE'
                self.reference_return_deferred = self.state == 'STOP_EXIT'
                self._enter('DONE', now)

    def _wait_arm(self, now, arm):
        if arm is None or arm.command_id != self.arm_request['command_id']:
            return
        if arm.state in ('FAILED', 'UNKNOWN'):
            raise RuntimeError(arm.reason)
        if arm.state != 'COMPLETED':
            return
        if self.state == 'STOW_ARM':
            self.base_locked = False
            self._enter('STOP_EXIT' if self.defer_reference_return else 'RETURN_REFERENCE', now)
        elif self.state == 'PICK':
            self._confirm_pick()
            self._next_item(now)
        elif self.state == 'RESTORE_WORK':
            self._next_item(now)
        else:
            self._enter('APPROACH', now)

    def _confirm_pick(self):
        self.confirmed.append(('PICK', self.item+1, self.colors[self.item], 0))
        self.item += 1

    def _next_item(self, now):
        self.visual_started = now
        self.tracked_color = None
        self._enter('WAIT_REQUIRED' if self.item < 3 else 'STOW_ARM', now)

    def _vision(self, now, obs, gantry, arm):
        if (obs is None or obs.kind != 'material' or
                not 0 <= now-obs.captured_at <= self.p['vision']['max_frame_age_s'] or
                obs.captured_at <= self.visual_started or
                obs.geometry_target not in (0, self.vision_target)):
            self.arm_velocity, self.servo_enabled, self.count = (0., 0.), False, 0
            self._gantry.reset()
            return
        fresh = self.last_sequence is None or obs.sequence > self.last_sequence
        if fresh:
            if self.last_observed_at is not None and obs.captured_at <= self.last_observed_at:
                raise ValueError('原料图像时间未递增')
            self.last_sequence, self.last_observed_at = obs.sequence, obs.captured_at
        points = dict(obs.materials)
        required = self.colors[self.item]
        if fresh and required in points:
            self._follow_required(now, obs, points[required], gantry)
        elif fresh:
            self.tracked_color = None
            self._enter('WAIT_REQUIRED', now)
            self.diagnostic = dict(required_color=required, qualified_frames=0, wait_reason='停在当前位置等待所需颜色')
        elif self.state == 'TRACK_REQUIRED' and self._gantry.active and self.p['motion_prediction']['enabled']:
            self._update_gantry(now, gantry, fresh=False)

    def _error(self, obs, point):
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise ValueError('目标中心坐标无效')
        q = self.p['vision']['target_uv_px'] or [(obs.image_size[0]-1)/2, (obs.image_size[1]-1)/2]
        return point[0]-q[0], point[1]-q[1]

    def _follow_required(self, now, obs, point, gantry):
        if self.state != 'TRACK_REQUIRED':
            self._enter('TRACK_REQUIRED', now)
        self.tracked_color = self.colors[self.item]
        if (self.p['motion_prediction']['enabled'] and self._gantry.observed_at is not None and
                obs.captured_at-self._gantry.observed_at > self.p['motion_prediction']['max_horizon_s']):
            self.count = 0
        self._gantry.observe(self._error(obs, point), obs.captured_at)
        self.diagnostic = dict(required_color=self.tracked_color, tracked_color=self.tracked_color, center=point,
                               wait_reason=f"跟随所需颜色；等待{self.p['grasp_gate']['qualified_frames']}帧中心与两轴输出达标")
        self._update_gantry(now, gantry, fresh=True)

    def _update_gantry(self, now, gantry, *, fresh):
        self.arm_velocity = self._gantry.update(now)
        self.servo_enabled = self._gantry.active
        self.count = self.count+int(fresh) if self._gantry.aligned else 0
        self.diagnostic.update(self._gantry.diagnostic, qualified_frames=self.count)
        if fresh and self.count >= self.p['grasp_gate']['qualified_frames']:
            self._enter('PICK', now)

