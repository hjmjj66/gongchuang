"""Real operations must wait for their own completion, with no simulated restore."""
from pathlib import Path
from types import SimpleNamespace
import unittest

from agv_perception.work_control import Pose
from coarse_debug.config import read_config
from coarse_debug.task import DebugTask, RawDebugTask, StorageDebugTask, parse_execution


class ExecutionTests(unittest.TestCase):
    def cases(self):
        config = read_config(Path(__file__).parents[1]/'config/debug.yaml')
        for area in ('coarse', 'storage'):
            config[area].pop('gantry_presets', None)
        return (
            (RawDebugTask(config['raw'], 'raw', (2, 1, 3), 0., execution='real'), 'PICK', 4, 'WAIT_REQUIRED'),
            (DebugTask(config['coarse'], 'place', (2, 1, 3), 0., execution='real'), 'PLACE', 6, 'ALIGN_RING'),
            (DebugTask(config['coarse'], 'pick', (2, 1, 3), 0., execution='real'), 'PICK', 5, 'RETURN_ITEM'),
            (StorageDebugTask(config['storage'], 'place', (2, 1, 3), 0., 1, {}, execution='real'), 'PLACE', 6, 'ALIGN_RING'),
            (StorageDebugTask(config['storage'], 'stack', (5, 1, 6), 0., 2, {1: 1, 5: 2, 6: 3}, execution='real'), 'STACK', 7, 'ALIGN_MATERIAL'),
        )

    def test_real_operations_wait_for_matching_completed_program(self):
        for task, state, command, next_state in self.cases():
            with self.subTest(area=task.task_id):
                task._enter(state, 0.)
                request = dict(task.arm_request)
                self.assertEqual(request['cmd'], command)
                pose = Pose(0., 0., 0., 2.)
                for status in (None,
                               SimpleNamespace(command_id='old-request', state='COMPLETED'),
                               SimpleNamespace(command_id=request['command_id'], state='RUNNING')):
                    task.tick(2., pose, arm=status)
                    self.assertEqual(task.state, state)
                    self.assertEqual(task.arm_request, request)
                    self.assertEqual(task.confirmed, [])
                    self.assertFalse(task.servo_enabled)
                    self.assertEqual(task.arm_velocity, (0., 0.))
                task.tick(2., pose, arm=SimpleNamespace(command_id=request['command_id'], state='COMPLETED'))
                self.assertEqual(task.state, next_state)
                self.assertEqual(task.item, 1)
                self.assertEqual(len(task.confirmed), 1)
                self.assertIsNone(task.arm_request)  # No simulation-only work-pose request.

    def test_real_operation_failure_and_stop_never_confirm_item(self):
        for failure in ('FAILED', 'UNKNOWN', 'stop'):
            for task, state, _, _ in self.cases():
                with self.subTest(area=task.task_id, failure=failure):
                    task._enter(state, 0.)
                    status = SimpleNamespace(command_id=task.arm_request['command_id'], state=failure, reason='program failed')
                    if failure == 'stop':
                        task.stop(.5, 'stop', failed=False)
                        status.state = 'COMPLETED'
                        task.tick(2., Pose(0., 0., 0., 2.), arm=status)
                        self.assertEqual(task.state, 'STOPPED')
                        self.assertIsNone(task.arm_request)
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'program failed'):
                            task.tick(2., Pose(0., 0., 0., 2.), arm=status)
                    self.assertEqual(task.confirmed, [])

    def test_missing_execution_defaults_to_simulation_and_invalid_values_reject(self):
        self.assertEqual(parse_execution({}), 'simulated')
        for execution in ('simulated', 'real'):
            self.assertEqual(parse_execution(dict(execution=execution)), execution)
        for execution in (None, True, 1, '', 'Real', '1s', [], {}):
            with self.subTest(execution=execution), self.assertRaises(ValueError):
                parse_execution(dict(execution=execution))
