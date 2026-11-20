"""Check shared settings through real control outputs and persisted reloads."""
from copy import deepcopy
from pathlib import Path
import unittest

from agv_perception.coarse_control import CoarseTask
from agv_perception.gantry_presets import PresetMove
from agv_perception.raw_control import RawTask
from agv_perception.work_control import GantryPose, Observation, Pose
from coarse_debug.config import area_fields, fixed_parameters, parameter_groups, read_config
from coarse_debug.tracking import TrackingTask

CONFIG = Path(__file__).parents[1]/'config/debug.yaml'


class SharedMotionTests(unittest.TestCase):
    def setUp(self):
        self.config = read_config(CONFIG)

    def test_both_regions_and_storage_visits_use_same_visual_motion(self):
        outputs = []
        for area, visit in (('coarse', 1), ('storage', 1), ('storage', 2)):
            p = deepcopy(self.config[area])
            p['vision'].update(target_uv_px=[100., 100.], stack_target_uv_px=[100., 100.])
            task = TrackingTask(p, area, 2, 'material' if visit == 2 else 'ring', 0., area, visit)
            obs = Observation(task.kind, 1, .1, (201, 201), {2: (130., 120.)}, ((2, (130., 120.)),))
            task.tick(.1, observation=obs)
            outputs.append(task.arm_velocity)
        self.assertTrue(any(outputs[0]))
        self.assertEqual(outputs, [outputs[0]]*3)

    def test_preposition_and_retrieval_apply_same_pid_including_i_and_d(self):
        p = deepcopy(self.config['coarse'])
        c = p['gantry_motion']
        c.update(max_yaw_rad_s=10., max_fb_rpm=100., timeout_s=30.,
                 angle_tolerance_rad=.0001, length_tolerance_pulses=.01,
                 yaw_pid=dict(kp=.5, ki=.1, kd=.02), length_pid=dict(kp=.2, ki=.1, kd=.02))
        target = dict(fb_pulses=50, yaw_rad=.5)
        move = PresetMove(c, target, 0., .2)
        task = CoarseTask(p, 'return', (1,), (1,), 0.)
        task.placements[1] = (target['fb_pulses'], target['yaw_rad'])
        task._enter('RETURN_ITEM', 0.)
        task.return_started = 0.
        for now, length, angle in ((.1, 10, .1), (.2, 15, .15), (.3, 20, .2)):
            gantry = GantryPose(length, angle, now)
            good, velocity, _ = move.update(now, gantry)
            self.assertFalse(good)
            task.tick(now, Pose(0., 0., 0., now), gantry=gantry)
            self.assertEqual(task.arm_velocity, velocity)
        self.assertGreater(move.pids[0].integral, 0.)

    def test_raw_waiting_never_uses_shared_gantry_return(self):
        p = deepcopy(self.config['raw'])
        task = RawTask(p, 'raw', (1, 2, 3), 0.)
        task._enter('WAIT_REQUIRED', 0.)
        for i in range(1, 4):
            now = i*.05
            task.tick(now, Pose(0., 0., 0., now),
                      Observation('material', i, now, (201, 201), {}, ()),
                      gantry=GantryPose(100*i, .1*i, now))
            self.assertEqual(task.arm_velocity, (0., 0.))
            self.assertEqual(task.state, 'WAIT_REQUIRED')

    def test_web_schema_has_one_motion_group_and_explicit_scope(self):
        for area in ('coarse', 'storage'):
            paths = {f['path'] for f in area_fields(area)}
            self.assertIn('gantry_motion.yaw_pid.kd', paths)
            self.assertIn('gantry_motion.length_tolerance_pulses', paths)
            self.assertFalse(any(p.startswith(('gantry_return.', 'gantry_presets.')) for p in paths))
            for group in ('angle_alignment', 'base_servo', 'gantry_servo', 'gantry_motion'):
                self.assertIn('三区域共用' if group=='gantry_motion' else '粗加工 + 暂存共用', parameter_groups(area)[group]['label'])
        raw_paths = {f['path'] for f in area_fields('raw')}
        self.assertFalse(any(p.startswith('initial_follow.') and p != 'initial_follow.duration_s' for p in raw_paths))

    def test_web_covers_all_numeric_controls_and_explains_fixed_rules(self):
        def numeric_paths(values, prefix=''):
            for key, value in values.items():
                path = prefix+key
                if isinstance(value, dict):
                    yield from numeric_paths(value, path+'.')
                elif isinstance(value, (int, float)):
                    yield path

        for area in ('raw', 'coarse', 'storage'):
            with self.subTest(area=area):
                editable = {f['path'] for f in area_fields(area)}
                fixed = {f['path'] for f in fixed_parameters(self.config, area)}
                controls = {p for p in numeric_paths(self.config[area]) if not p.startswith('gantry_presets.')}
                self.assertEqual(editable & fixed, set())
                self.assertLessEqual(controls, editable | fixed)
                groups = parameter_groups(area)
                for path in controls:
                    details = groups[path.split('.')[0]]
                    self.assertTrue(details['label'])
                    self.assertTrue(details['hint'])
                self.assertIn('execution.control_rate_hz', fixed)
