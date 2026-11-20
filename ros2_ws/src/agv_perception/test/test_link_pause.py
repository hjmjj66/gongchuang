"""Use actual region tasks to verify deadlines and visual history on reconnection."""
from pathlib import Path
import unittest

from agv_perception.coarse_control import CoarseTask
from agv_perception.raw_control import RawTask
from agv_perception.storage_control import StorageTask
from agv_perception.work_config import read_work_config
from agv_perception.link_pause import resume_task
from agv_perception.work_control import PID


class LinkPauseTests(unittest.TestCase):
    def test_region_tasks_keep_steps_and_gain_no_elapsed_timeout(self):
        config = read_work_config(Path(__file__).resolve().parents[4]/'tools/coarse_debug/config/debug.yaml')
        tasks = (CoarseTask(config['coarse'], 'c', [1, 2, 3], [1, 2, 3], 10.),
                 RawTask(config['raw'], 'r', [1, 2, 3], 10.),
                 StorageTask(config['storage'], 's', [1, 2, 3], [1, 2, 3], 10., 'STORAGE_PLACE', {}))
        for task in tasks:
            with self.subTest(task=type(task).__name__):
                state, request = task.state, task.arm_request
                task.last_sequence, task.last_observed_at = 4, 10.1
                pid = PID(dict(kp=1., ki=1., kd=1.), 1.)
                pid.update(1., 10.)
                pid.update(1., 10.1)
                task._pids = [pid]
                resume_task(task, 20.)
                self.assertEqual(task.state, state)
                self.assertEqual(task.arm_request, request)
                self.assertEqual(task.entered_at, 30.)
                self.assertIsNone(task.last_observed_at)
                self.assertIsNone(pid.at)
                self.assertEqual(pid.integral, 0.)
