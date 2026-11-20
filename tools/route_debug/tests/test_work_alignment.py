"""Exercise the station-only task with real observations and odometry."""

from agv_perception.work_config import read_work_config

import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import yaml

from agv_perception.coarse_control import Observation, Pose
from route_debug.work_alignment import WorkAlignmentTask


class WorkAlignmentTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[3]
        self.config = read_work_config((root/'tools/coarse_debug/config/debug.yaml'))
        self.config.pop('motion_config', None)  # Isolated expanded simulation settings.
        self.config.pop('motion_config', None)  # Isolated expanded simulation settings.
        self.now = 0.

    def step(self, task, *, complete=False, xy=(0., 0.), yaw=math.pi/2, rings=None, speed=0., dt=.05):
        self.now += dt
        arm = SimpleNamespace(command_id='', state='IDLE', idle=True, pending=False)
        if complete:
            arm.command_id, arm.state = task.arm_request['command_id'], 'COMPLETED'
        obs = Observation('ring', round(self.now*10000), self.now, (201, 201), rings or {})
        return task.tick(self.now, Pose(*xy, yaw, self.now, speed), obs, arm)

    def test_work_align_stow_then_return_to_p0_at_nonzero_heading(self):
        for area in ('coarse', 'storage'):
            with self.subTest(area=area):
                self.now = 0.
                p = self.config[area]
                p['vision']['target_uv_px'] = [100., 100.]
                task = WorkAlignmentTask(p, area, self.now, area=area)
                self.assertEqual(task.arm_request['cmd'], 3)
                self.step(task)
                self.assertEqual(task.state, 'WORK_ENTRY')
                self.step(task, complete=True)
                self.assertEqual(task.state, 'ALIGN_ENTRY')
                rings = {1:(20., 100.), 2:(100., 100.), 3:(180., 100.)}
                for _ in range(30):
                    self.step(task, rings=rings)
                    if task.state in ('WORK_EXIT', 'STOW'):
                        break
                self.assertEqual(task.entry_alignment, 'ALIGNED')
                if area == 'coarse':
                    self.assertEqual(task.state, 'WORK_EXIT')
                    self.assertEqual(task.arm_request['cmd'], 3)
                    self.step(task, complete=True)
                    self.assertEqual(task.state, 'ALIGN_EXIT')
                    self.step(task, rings={1:(20., 90.), 2:(100., 100.), 3:(180., 110.)})
                    self.assertEqual(task.state, 'ALIGN_EXIT')
                    self.assertNotEqual(task.velocity[2], 0.)
                    for _ in range(20):
                        self.step(task, rings=rings)
                        if task.state == 'STOW':
                            break
                    self.assertEqual(task.final_alignment, 'ALIGNED')
                else:
                    self.assertEqual(task.final_alignment, 'NOT_RUN')
                self.assertEqual(task.state, 'STOW')
                self.assertEqual(task.arm_request['cmd'], 2)
                self.assertEqual(task.velocity, (0., 0., 0.))
                self.step(task, xy=(0., .5))
                self.assertEqual(task.state, 'STOW')
                self.step(task, complete=True, xy=(.1, -.1))
                self.assertEqual(task.state, 'RETURN_REFERENCE')
                self.assertEqual(task.reference, (0.,0.))
                self.step(task, xy=(.1,-.1))
                self.assertGreater(task.velocity[0], 0.)
                self.assertGreater(task.velocity[1], 0.)
                self.assertEqual(task.velocity[2], 0.)
                for _ in range(10):
                    self.step(task, xy=(.1,-.1))
                self.assertEqual(task.state, 'RETURN_REFERENCE')
                self.step(task, speed=.02)
                self.step(task, speed=.02, dt=.2)
                self.assertEqual(task.state, 'RETURN_REFERENCE')
                self.step(task)
                self.assertEqual(task.state, 'DONE')
                self.assertTrue(task.returned)
                self.assertEqual(task.confirmed, [])
                self.assertEqual(task.placements, {})

    def test_already_at_p0_waits_for_stop_and_cancel_never_returns(self):
        task = WorkAlignmentTask(self.config['coarse'], 'zero', 0.)
        task.reference = (0.,0.)
        task._enter('STOW', self.now)
        self.step(task, complete=True)
        self.assertEqual(task.state, 'RETURN_REFERENCE')
        self.step(task)
        self.step(task, dt=.2)
        self.assertEqual(task.state, 'DONE')
        task = WorkAlignmentTask(self.config['coarse'], 'cancel', self.now)
        task.stop(self.now, '用户停止', failed=False)
        self.assertEqual(self.step(task), (0., 0., 0.))
        self.assertEqual(task.state, 'STOPPED')
