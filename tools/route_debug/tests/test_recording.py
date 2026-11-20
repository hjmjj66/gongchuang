"""Verify streamed run files and their metadata survive completion and restart."""

import json
from pathlib import Path
import tempfile
import unittest

from route_debug.recording import RunRecorder
from route_debug.session import Session
from test_session import CONFIG, Clock, IO


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'data'
        self.clock = Clock()
        self.recorder = RunRecorder(self.path, self.clock)
        self.session = Session(IO(), CONFIG, {}, clock=self.clock)

    def read_rows(self):
        return [json.loads(line) for line in (self.path/self.recorder.name).read_text().splitlines()]

    def test_streamed_samples_are_readable_before_finish(self):
        self.assertFalse(self.path.exists())
        self.assertEqual(self.recorder.status()['files'], [])
        self.recorder.start({'translation_pid_kp': 1.8}, {'mode': 'free'})
        self.addCleanup(self.recorder.finish, 'STOPPED', 'cleanup')
        for _ in range(3):
            self.clock.now += .05
            self.recorder.sample(self.session.snapshot(), {'progress_receive_s': None})
        rows = self.read_rows()
        self.assertEqual(rows[0]['type'], 'metadata')
        self.assertEqual(rows[0]['sample_interval_s'], .05)
        samples = [row for row in rows if row['type'] == 'sample']
        self.assertEqual(len(samples), 3)
        self.assertAlmostEqual(samples[1]['t_s']-samples[0]['t_s'], .05)
        self.assertEqual(len([row for row in rows if row['type'] == 'event']), 1)
        self.assertIsNone(samples[0]['odometry'])
        self.assertIsNone(samples[0]['feedback_ages_s']['progress_receive_s'])
        self.assertTrue(self.recorder.status()['active'])

    def test_summary_multiple_runs_and_file_list_survive_restart(self):
        names = []
        for state in ('FINISHED', 'STOPPED', 'FAULT'):
            self.recorder.start({'angle_pid_kp': 1.}, None)
            self.session._state(state, 'test reason')
            self.recorder.sample(self.session.snapshot(), {})
            self.recorder.finish(state, 'test reason')
            names.append(self.recorder.name)
            self.assertEqual(self.read_rows()[-1]['state'], state)
            self.assertEqual(self.read_rows()[-1]['samples'], 1)
            self.assertFalse(self.recorder.status()['active'])
        recorder = RunRecorder(self.path)
        self.assertCountEqual([file['name'] for file in recorder.status()['files']], names)
        self.assertTrue(all(file['url'].startswith('/api/recordings/route_')
                            for file in recorder.status()['files']))

    def test_bad_sample_stops_recording_with_visible_error(self):
        self.recorder.start({}, {})
        try:
            self.session.last_command = [float('nan'), 0., 0.]
            self.recorder.sample(self.session.snapshot(), {})
        except ValueError as exc:
            self.recorder.failed(exc)
        self.assertFalse(self.recorder.status()['active'])
        self.assertIn('数据记录失败', self.recorder.status()['error'])
        self.assertEqual(self.read_rows()[0]['type'], 'metadata')

    def test_previous_run_events_are_excluded_and_screen_settings_are_kept(self):
        boundary = self.session.events[-1]
        self.recorder.start({}, {'mode': 'screen'}, boundary)
        self.session._record('new run event')
        self.session.settings = dict(mode='screen', start=3, goal=1, blocked_edges=[[2, 3]])
        self.recorder.sample(self.session.snapshot(), {})
        self.recorder.finish('FINISHED', '')
        rows = self.read_rows()
        self.assertEqual([row['message'] for row in rows if row['type'] == 'event'], ['new run event'])
        settings = next(row['settings'] for row in rows if row['type'] == 'settings')
        self.assertEqual(settings['start'], 3)
        self.assertEqual(settings['blocked_edges'], [[2, 3]])
