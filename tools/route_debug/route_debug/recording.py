"""Stream one JSONL file per run, without retaining samples in memory."""

from datetime import datetime
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo


SAMPLE_INTERVAL_S = .05


class RunRecorder:
    def __init__(self, directory, clock=time.monotonic):
        self.directory = Path(directory)
        self.clock = clock
        self.stream = None
        self.name = ''
        self.samples = 0
        self.error = ''
        self.last_event = None
        self.last_plan = None

    def start(self, config, settings, previous_event=None):
        self.directory.mkdir(parents=True, exist_ok=True)
        now = datetime.now(ZoneInfo('Asia/Shanghai'))
        self.name = now.strftime('route_%Y%m%d_%H%M%S_%f.jsonl')
        self.samples = 0
        self.error = ''
        self.last_event = previous_event
        self.last_plan = None
        self.last_settings = json.dumps(settings, sort_keys=True)
        self.started = self.clock()
        self.stream = (self.directory / self.name).open('x', encoding='utf-8', buffering=1)
        self._write(dict(type='metadata', schema_version=1, started_at=now.isoformat(),
                         sample_interval_s=SAMPLE_INTERVAL_S, config=config, settings=settings,
                         units=dict(position='m', position_error='m', yaw='deg',
                                    command=['m/s', 'm/s', 'deg/s'],
                                    measured_angular_velocity='rad/s'),
                         position_error_frame='fixed axes at segment.baseline_yaw_deg'))

    def _write(self, row):
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')

    def sample(self, state, feedback_ages):
        elapsed = self.clock() - self.started
        events = state['events']
        new_events = events
        if self.last_event in events:
            new_events = events[events.index(self.last_event)+1:]
        for event in new_events:
            self._write(dict(type='event', t_s=elapsed, **event))
        if events:
            self.last_event = events[-1]
        settings_key = json.dumps(state['settings'], sort_keys=True)
        if settings_key != self.last_settings:
            self._write(dict(type='settings', t_s=elapsed, settings=state['settings']))
            self.last_settings = settings_key
        plan_key = json.dumps(state['plan'], sort_keys=True)
        if plan_key != self.last_plan:
            self._write(dict(type='plan', t_s=elapsed, plan=state['plan']))
            self.last_plan = plan_key
        row = {key: state[key] for key in ('state', 'phase', 'reason', 'run_id',
               'route_index', 'goal', 'parking_maneuver', 'command', 'odometry', 'segment',
               'board_fresh', 'odometry_fresh', 'odom_session_id', 'motion_epoch', 'work')}
        row.update(type='sample', t_s=elapsed, feedback_ages_s=feedback_ages,
                   board={key: (state['board'] or {}).get(key) for key in
                          ('connected', 'status_valid', 'pc_control', 'odometry_valid',
                           'motion_allowed', 'fault_reason')})
        self._write(row)
        self.samples += 1

    def finish(self, state, reason):
        if self.stream is None:
            return
        try:
            self._write(dict(type='summary', duration_s=self.clock()-self.started,
                             samples=self.samples, state=state, reason=reason))
        finally:
            self.stream.close()
            self.stream = None

    def failed(self, error):
        self.error = '数据记录失败：' + str(error)
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def status(self):
        files = sorted(self.directory.glob('route_*.jsonl'), reverse=True)[:20]
        return dict(active=self.stream is not None, file=self.name, samples=self.samples,
                    interval_ms=50, error=self.error, directory=str(self.directory),
                    files=[dict(name=path.name, size_bytes=path.stat().st_size,
                                url='/api/recordings/'+path.name) for path in files])
