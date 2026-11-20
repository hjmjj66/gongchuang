"""Task stages select refinement targets and reject another target's observations."""

from dataclasses import replace
import unittest

from agv_perception.coarse_control import CoarseTask
from agv_perception.raw_control import RawTask
from agv_perception.storage_control import StorageTask
from agv_perception.work_control import Observation, Pose
from test_coarse_control import calibrated_config
from test_raw_control import calibrated_raw_config
from test_storage_control import calibrated_storage_config


class GeometryTargetTests(unittest.TestCase):
    def test_coarse_and_storage_ring_stages_keep_all_geometry_until_item_alignment(self):
        p = calibrated_config()
        p.pop('gantry_presets', None)
        tasks = [CoarseTask(p, 'coarse', [1, 5, 6], [2, 1, 3], 0.),
                 StorageTask(calibrated_storage_config(), 'storage', [1, 5, 6], [2, 1, 3],
                             0., 'STORAGE_PLACE', {})]
        for task in tasks:
            for state in ('ALIGN_ENTRY', 'CENTER_RING2', 'ALIGN_RING'):
                task._enter(state, 1.)
                self.assertEqual(task.vision_target, 2 if state == 'ALIGN_RING' else 0)
            task.item = 1
            task._enter('ALIGN_RING', 2.)
            self.assertEqual(task.vision_target, 1)
            wrong = Observation('ring', 1, 2.1, (201, 201), {1:(100.,100.)}, geometry_target=2)
            task.tick(2.1, Pose(0.,0.,0.,2.1), wrong)
            self.assertEqual(task.count, 0)
            self.assertEqual(task.arm_velocity, (0.,0.))
            task.tick(2.2, Pose(0.,0.,0.,2.2), replace(wrong, sequence=2, captured_at=2.2, geometry_target=1))
            self.assertEqual(task.count, 1)
            task._enter('ALIGN_ENTRY', 3.)
            task.tick(3.1, Pose(0.,0.,0.,3.1), replace(wrong, captured_at=3.1))
            self.assertEqual(task.count, 0)

    def test_material_scan_and_ordered_follow_choose_different_refinement(self):
        raw = RawTask(calibrated_raw_config(), 'raw', [1, 3, 4], 0.)
        for state in raw.INITIAL_STATES:
            raw._enter(state, 1.)
            self.assertEqual(raw.vision_target, 0)
        for state in ('WAIT_REQUIRED', 'TRACK_REQUIRED'):
            raw._enter(state, 1.)
            self.assertEqual(raw.vision_target, 1)
        stack = StorageTask(calibrated_storage_config(), 'stack', [5, 1, 6], [2, 1, 3],
                            0., 'STORAGE_STACK', {1:1,5:2,6:3})
        stack._enter('SCAN_FIRST_LAYER', 1.)
        self.assertEqual(stack.vision_target, 0)
        stack._enter('ALIGN_MATERIAL', 2.)
        self.assertEqual(stack.vision_target, 5)
        for task, expected in ((raw, 1), (stack, 5)):
            obs = Observation('material', 1, 2.1, (201,201), {}, ((expected,(100.,100.)),),
                              geometry_target=6)
            task.tick(2.1, Pose(0.,0.,0.,2.1), obs)
            self.assertEqual(task.count, 0)
            task.tick(2.2, Pose(0.,0.,0.,2.2), replace(obs, sequence=2, captured_at=2.2,
                                                     geometry_target=expected))
            self.assertEqual(task.count, 1)


if __name__ == '__main__':
    unittest.main()
