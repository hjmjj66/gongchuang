"""Reference adjustment and pure visual tracking for gantry preset calibration."""

from agv_perception.coarse_control import CoarseTask

from .tracking import TrackingTask


class ReferenceAdjustment(CoarseTask):
    CENTER_NEXT_STATE = 'DONE'

    def __init__(self, config, task_id, now):
        super().__init__(config, task_id, (2,), (2,), now)

    def _angle_done(self, now, outcome):
        if outcome != 'ALIGNED':
            raise RuntimeError('角度纠偏未达标，请重新点击调整后再标定')
        super()._angle_done(now, outcome)


class PoseCalibrationTracking(TrackingTask):
    def __init__(self, config, task_id, target, now, area):
        super().__init__(config, task_id, target, 'ring', now, area)
        self.qualified_frames = 0
        self.saved = False
        self.sample = None

    def request_action(self, now):
        raise ValueError('机构初值标定只追踪与保存，不执行取放')

    def tick(self, now, pose=None, observation=None, arm=None, gantry=None, arm_at=None):
        previous = self.last_sequence
        result = super().tick(now, pose, observation, arm, gantry, arm_at)
        if self.state != 'TRACK_HOLD' or not self.servo_enabled:
            self.qualified_frames = 0
        elif self.last_sequence != previous:
            self.qualified_frames += 1
        self.diagnostic['qualified_frames'] = self.qualified_frames
        return result

    def request_save(self, gantry):
        self._zero()
        self.sample = dict(fb_pulses=gantry.fb_pulses, yaw_rad=gantry.yaw_rad)
        self.state, self.reason = 'STOPPED', ''
