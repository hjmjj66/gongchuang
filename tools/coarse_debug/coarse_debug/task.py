"""Use competition controllers with optional one-second pick/place simulation."""

from agv_perception.coarse_control import CoarseTask
from agv_perception.storage_control import StorageTask
from agv_perception.raw_control import RawTask


def parse_execution(settings):
    execution = settings.get('execution', 'simulated')
    if execution not in ('simulated', 'real'):
        raise ValueError('取放执行方式须为1秒模拟或真实执行')
    return execution


class DebugTask(CoarseTask):
    ARM_STATES = dict(CoarseTask.ARM_STATES, DEBUG_RESTORE_WORK='work_pose_cmd')
    def __init__(self, config, task_id, positions, now, dwell_s=1., *, execution='simulated', colors=None):
        self.execution = parse_execution(dict(execution=execution))
        self.dwell_s = dwell_s
        super().__init__(config, task_id, positions if colors is None else colors, positions, now)

    def _enter(self, state, now):
        super()._enter(state, now)
        if self.execution == 'simulated' and state in ('PLACE', 'PICK'):
            self.arm_request = None

    def _wait_arm(self, now, arm):
        if self.execution == 'real' or self.state not in ('PLACE', 'PICK'):
            return super()._wait_arm(now, arm)
        if now-self.entered_at < self.dwell_s:
            return
        if self.state == 'PLACE':
            self._arm_completed(now)
            return
        self.simulated_op = self.state
        self._enter('DEBUG_RESTORE_WORK', now)
        self.arm_request['command_id'] = f'{self.task_id}:restore:{self.simulated_op}:{self.item}'

    def _arm_completed(self, now):
        if self.state == 'DEBUG_RESTORE_WORK':
            self.state = self.simulated_op
        super()._arm_completed(now)


class StorageDebugTask(StorageTask):
    def __init__(self, config, task_id, order, now, visit, first_layer, *, execution='simulated', colors=None):
        self.execution = parse_execution(dict(execution=execution))
        self.dwell_s = 1.
        operation = 'STORAGE_PLACE' if visit == 1 else 'STORAGE_STACK'
        positions = order if visit == 1 else tuple(first_layer[color] for color in order)
        super().__init__(config, task_id, order if colors is None else colors, positions, now, operation, first_layer)

    def _enter(self, state, now):
        super()._enter(state, now)
        if self.execution == 'simulated' and state in ('PLACE', 'STACK'):
            self.arm_request = None

    def _wait_arm(self, now, arm):
        if self.execution == 'real' or self.state not in ('PLACE', 'STACK'):
            return super()._wait_arm(now, arm)
        if now-self.entered_at >= self.dwell_s:
            self._arm_completed(now)


class RawDebugTask(RawTask):
    def __init__(self, config, task_id, colors, now, *, execution='simulated'):
        self.execution = parse_execution(dict(execution=execution))
        self.dwell_s = 1.
        self.placements = {}
        self.entry_alignment = 'NOT_RUN'
        super().__init__(config, task_id, colors, now)

    def _enter(self, state, now):
        super()._enter(state, now)
        if self.execution == 'simulated' and state == 'PICK':
            self.arm_request = None

    def _wait_arm(self, now, arm):
        if self.execution == 'real' or self.state != 'PICK':
            super()._wait_arm(now, arm)
        elif now-self.entered_at >= self.dwell_s:
            self._confirm_pick()
            self._enter('RESTORE_WORK', now)
