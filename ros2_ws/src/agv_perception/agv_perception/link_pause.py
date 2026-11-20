"""Freeze task deadlines while the serial link recovers."""


def resume_task(task, duration):
    for name in ('entered_at', 'visual_started', 'initial_started', 'return_started',
                 'zero_since', 'search_started', 'missing_since', '_alignment_started',
                 'scan_at', 'scan_deadline', 'action_at'):
        value = getattr(task, name, None)
        if value is not None:
            setattr(task, name, value + duration)
    task.last_sequence = task.last_observed_at = None
    task.count = 0
    task.velocity, task.arm_velocity = (0., 0., 0.), (0., 0.)
    for pid in getattr(task, '_pids', ()):
        pid.reset()
    for name in ('preset_move', 'return_move'):
        move = getattr(task, name, None)
        if move is not None:
            move.started_at += duration
            for pid in move.pids:
                pid.reset()
    for name in ('_gantry', '_raw_servo'):
        servo = getattr(task, name, None)
        if servo is not None:
            servo.reset()
