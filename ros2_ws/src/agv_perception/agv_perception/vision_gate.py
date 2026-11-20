"""Fresh observations and startup waits shared by all regional task hosts."""


class VisionGate:
    def __init__(self):
        self.key = None

    def filter(self, task, obs, now, area):
        if not task.vision_kind:
            return None
        pose_at = task.visual_started if area == 'raw' else task.entered_at
        key = (task.task_id, task.vision_kind, task.vision_target, pose_at)
        if key != self.key:
            self.key, self.started_at = key, now
        p = task.p['vision']
        current = (obs is not None and obs.kind == task.vision_kind and
                   (obs.geometry_target == task.vision_target or
                    obs.geometry_target == 0 and obs.captured_at > self.started_at) and
                   obs.captured_at > pose_at)
        if current and 0 <= now-obs.captured_at <= p['max_frame_age_s']:
            return obs
        task.velocity, task.arm_velocity = (0., 0., 0.), (0., 0.)
        task.servo_enabled, task.count = False, 0
        if area == 'raw':
            task._gantry.reset()
        else:
            task._missing()
        task.diagnostic['wait_reason'] = '识别结果过期，停轴等待新鲜图像'
        return None
