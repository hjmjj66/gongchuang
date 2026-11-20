"""Storage: first-visit chassis calibration, gantry placement, stow and return to P0."""

import math


from .coarse_control import CoarseTask, validate_exit_motion_config, validate_gantry_work_config
from .work_config import read_work_config
from .work_control import PID, target_uv


def read_storage_config(path):
    return read_work_config(path)['storage']


def validate_storage_config(p):
    validate_gantry_work_config(p, ('work_pose_cmd', 'place_cmd', 'stack_cmd', 'stow_cmd'), with_lower=False)
    validate_exit_motion_config(p)
    count = p['ring_search']['scan_settle_frames']
    if type(count) is not int or count < 1:
        raise ValueError('scan_settle_frames 必须为正整数')
    point = p['vision'].get('stack_target_uv_px', p['vision']['target_uv_px'])
    if (not isinstance(point, list) or len(point) not in (0, 2) or
            any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in point)):
        raise ValueError('stack_target_uv_px 须为空或非负有限的 [u,v]')


class StorageTask(CoarseTask):
    CENTER_NEXT_STATE = 'ALIGN_RING'
    ARM_STATES = {'WORK_ENTRY': 'work_pose_cmd', 'SCAN_RESTORE_WORK': 'work_pose_cmd',
                  'PLACE': 'place_cmd', 'STACK': 'stack_cmd', 'STOW': 'stow_cmd'}
    ANGLE_STATES = ('ALIGN_ENTRY',)
    MATERIAL_STATES = ('SCAN_FIRST_LAYER', 'ALIGN_MATERIAL')

    def __init__(self, config, task_id, colors, positions, now, operation, first_layer, *,
                 defer_reference_return=False):
        self.operation = operation
        self.GANTRY_NEXT_STATE = 'PLACE' if operation == 'STORAGE_PLACE' else 'STACK'
        self.first_layer = dict(first_layer)
        self.seen_colors = set()
        self.scan_counts = dict.fromkeys(colors, 0)
        super().__init__(config, task_id, colors, positions, now,
                         defer_reference_return=defer_reference_return)

    @property
    def vision_kind(self):
        return 'material' if self.state in self.MATERIAL_STATES else super().vision_kind

    @property
    def vision_target(self):
        return self.colors[self.item] if self.state == 'ALIGN_MATERIAL' else super().vision_target

    @property
    def base_locked(self):
        return (self.operation == 'STORAGE_STACK' or
                self.state not in ('WORK_ENTRY', 'ALIGN_ENTRY', 'CAPTURE_REFERENCE', 'CENTER_RING2', 'RETURN_REFERENCE'))

    def _enter(self, state, now):
        super()._enter(state, now)
        self.target_seen = False
        if state == 'ALIGN_MATERIAL':
            c = self.p['gantry_servo']
            self._pids = [PID(c['u_pid'], c['max_yaw_rad_s']), PID(c['v_pid'], c['max_fb_rpm'])]
        if state == 'SCAN_FIRST_LAYER':
            self.scan_offset = 0.
            self.scan_at = now
            self.scan_target = None
            self.scan_explored = set()
        self.diagnostic = {}

    def tick(self, now, pose, observation=None, arm=None, gantry=None):
        if self.state == 'SCAN_FIRST_LAYER':
            # Bound physical search time at the control rate, even between camera frames.
            if self.arm_velocity[0]:
                self.scan_offset += self.arm_velocity[0]*(min(now,self.scan_deadline)-self.scan_at)
                if now >= self.scan_deadline:
                    self.scan_explored.add(self.scan_target)
                    self.scan_target = None
                    self._missing()
            self.scan_at = now
        elif (self.search_started is not None and not self.target_seen and
              now-self.search_started >= self.p['ring_search']['duration_s']):
            self._missing()
        return super().tick(now,pose,observation,arm,gantry)

    def _arm_completed(self, now):
        if self.state in ('PLACE', 'STACK'):
            self.confirmed.append((self.state, self.item+1, self.colors[self.item], self.positions[self.item]))
            self.item += 1
            self._enter('STOW' if self.item == 3 else self._item_state(), now)
        elif self.state == 'WORK_ENTRY':
            self._enter('ALIGN_ENTRY' if self.operation == 'STORAGE_PLACE' else 'SCAN_FIRST_LAYER', now)
        else:
            self._enter({'SCAN_RESTORE_WORK': 'ALIGN_MATERIAL',
                         'STOW': ('STOP_EXIT' if self.defer_reference_return else 'RETURN_REFERENCE')
                         if self.operation == 'STORAGE_PLACE' else 'DONE'}[self.state], now)

    def _item_state(self):
        return 'ALIGN_RING' if self.operation == 'STORAGE_PLACE' else 'ALIGN_MATERIAL'

    def _vision(self, now, pose, obs):
        if self.state == 'ALIGN_ENTRY' and now-self.entered_at >= self.p['angle_alignment']['timeout_s']:
            self._angle_done(now, 'TIMEOUT_SKIPPED')
            return
        fresh_pose = (obs is not None and obs.kind == self.vision_kind and obs.captured_at > self.entered_at and
                      obs.geometry_target in (0, self.vision_target))
        if not fresh_pose or not 0 <= now-obs.captured_at <= self.p['vision']['max_frame_age_s']:
            self._missing()
            return
        if obs.sequence == self.last_sequence:
            return
        if self.last_observed_at is not None and obs.captured_at-self.last_observed_at > self.p['vision']['max_frame_age_s']:
            self._missing()
            self.scan_counts = dict.fromkeys(self.scan_counts, 0)
        self.last_sequence, self.last_observed_at = obs.sequence, obs.captured_at
        if self.state == 'ALIGN_ENTRY':
            self._angle(now, obs)
            return
        if self.state == 'SCAN_FIRST_LAYER':
            self._scan(now, obs)
            return
        ring = 2 if self.state == 'CENTER_RING2' else self.positions[self.item]
        color = self.colors[self.item]
        point = (next((p for c, p in obs.materials if c == color), None)
                 if self.state == 'ALIGN_MATERIAL' else obs.rings.get(ring))
        if point is None:
            self._missing()
            self.diagnostic = dict(target_ring=ring, target_color=color,
                                   wait_reason='目标丢失，原地等待' if self.target_seen else '等待目标')
            if self.state != 'CENTER_RING2' and not self.target_seen and ring in (1, 3):
                c = self.p['ring_search']
                if self.search_started is None:
                    self.search_started = now
                if now-self.search_started < c['duration_s']:
                    self.arm_velocity = ((1 if ring == 1 else -1)*c['yaw_rad_s'], 0.)
                    self.servo_enabled = True
            return
        self.target_seen = True
        target = target_uv(self.p, obs.image_size, stack=self.state == 'ALIGN_MATERIAL')
        eu, ev = point[0]-target[0], point[1]-target[1]
        self.diagnostic = dict(target_ring=ring, target_color=color, center=point, target_uv=target, error_uv=(eu, ev))
        if self.state == 'CENTER_RING2':
            self._center(now, pose, obs, eu, ev)
        else:
            self._align_ring(now, obs, eu, ev)

    def _scan(self, now, obs):
        # Search offsets refer to the known WORK pose, not arbitrary image number ordering.
        visible = {color for color, _ in obs.materials if color in self.first_layer}
        for color in self.scan_counts:
            self.scan_counts[color] = self.scan_counts[color]+1 if color in visible else 0
            if self.scan_counts[color] >= self.p['ring_search']['scan_settle_frames']:
                self.seen_colors.add(color)
        self._missing()
        self.diagnostic = dict(seen_colors=sorted(self.seen_colors), visible_colors=sorted(visible),
                               wait_reason='等待第一层三色确认')
        if len(self.seen_colors) == 3:
            self._enter('SCAN_RESTORE_WORK', now)
            return
        if self.scan_target in self.seen_colors:
            self.scan_target = None
        if self.scan_target is None:
            self.scan_target = next((c for c in self.colors if c not in self.seen_colors and
                                     c not in self.scan_explored and self.first_layer[c] != 2), None)
            self.scan_direction = None
        if self.scan_target is None or self.scan_target in visible:
            return
        c = self.p['ring_search']
        target = (1 if self.first_layer[self.scan_target] == 1 else -1)*c['yaw_rad_s']*c['duration_s']
        remaining = target-self.scan_offset
        if self.scan_direction is None:
            self.scan_direction = 1 if remaining > 0 else -1
        if remaining*self.scan_direction <= c['yaw_rad_s']/self.p['execution']['control_rate_hz']:
            self.scan_explored.add(self.scan_target)
            self.scan_target = None
            return
        self.scan_deadline = now+abs(remaining)/c['yaw_rad_s']
        self.arm_velocity = (self.scan_direction*c['yaw_rad_s'], 0.)
        self.servo_enabled = True
        self.diagnostic.update(search_color=self.scan_target, scan_offset_rad=self.scan_offset)
