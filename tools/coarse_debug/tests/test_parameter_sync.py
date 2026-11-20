"""Parameter saves synchronize real YAML files and roll back failed commits."""

from motion_fixture import copy_debug_config
from pathlib import Path
import os
import tempfile
import unittest
from unittest.mock import patch

import yaml

from agv_perception.work_config import SHARED_GROUPS, read_work_config
from coarse_debug.config import area_fields, fixed_parameters, get_value, read_config, save_parameters, sync_parameters

ROOT = Path(__file__).resolve().parents[3]


class ParameterSyncTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.folder = Path(folder.name)
        self.debug = self.folder/'debug.yaml'
        copy_debug_config(self.debug)
        self.config = read_config(self.debug)
        self.paths = {}
        for area in ('raw', 'coarse', 'storage'):
            self.paths[area] = []
            for suffix in ('source', 'installed'):
                path = self.folder/f'{area}-{suffix}.yaml'
                text = (ROOT/f'ros2_ws/src/agv_perception/config/{area}.yaml').read_text()
                path.write_text(text+'\nunrelated: keep\n')
                self.paths[area].append(path)

    def test_save_each_area_syncs_full_parameters_preserves_other_areas_and_comments(self):
        for area in self.paths:
            before = read_config(self.debug)
            other_files = {p: p.read_bytes() for other, paths in self.paths.items() if other != area for p in paths}
            values = {f['path']: get_value(before, f['path'], area) for f in area_fields(area)}
            values['vision.target_uv_px'] = [100., 200.]
            self.config = save_parameters(self.debug, before, values, area, self.paths)
            self.assertEqual(read_config(self.debug), self.config)
            for other in set(self.paths)-{area}:
                self.assertEqual(self.config[other], before[other])
            for path, original in other_files.items():
                self.assertEqual(path.read_bytes(), original)
            for path in self.paths[area]:
                text = path.read_text()
                self.assertTrue(text.startswith('#'))
                data = yaml.safe_load(text)
                self.assertEqual(read_work_config(path)[area], self.config[area])
                self.assertEqual(data['unrelated'], 'keep')
        self.assertTrue(yaml.safe_load(self.paths['raw'][0].read_text())['raw']['motion_prediction']['enabled'])

    def test_initial_sync_and_symlinks_use_same_parameters(self):
        link = self.folder/'coarse-link.yaml'
        link.symlink_to(self.paths['coarse'][0])
        self.paths['coarse'].append(link)
        sync_parameters(self.debug, self.config, tuple(self.paths), self.paths)
        self.assertTrue(link.is_symlink())
        for area, paths in self.paths.items():
            for path in paths:
                self.assertEqual(read_work_config(path)[area], self.config[area])

    def test_raw_grasp_frames_are_editable_and_sync_to_formal_files(self):
        field = next(f for f in area_fields('raw') if f['path'] == 'grasp_gate.qualified_frames')
        self.assertTrue(field['integer'])
        self.assertNotIn('固定', field['label'])
        self.assertNotIn(field['path'], {f['path'] for f in fixed_parameters(self.config, 'raw')})
        values = {f['path']: get_value(self.config, f['path'], 'raw') for f in area_fields('raw')}
        values[field['path']] = 6
        saved = save_parameters(self.debug, self.config, values, 'raw', self.paths)
        self.assertEqual(read_config(self.debug)['raw']['grasp_gate']['qualified_frames'], 6)
        for path in self.paths['raw']:
            self.assertEqual(read_work_config(path)['raw']['grasp_gate']['qualified_frames'], 6)
        for area in ('coarse', 'storage'):
            self.assertEqual(saved[area], self.config[area])
        files = (self.debug, *self.paths['raw'])
        originals = {p: p.read_bytes() for p in files}
        for value in (0, -1, 2.5, True):
            values[field['path']] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                save_parameters(self.debug, saved, values, 'raw', self.paths)
            for path in files:
                self.assertEqual(path.read_bytes(), originals[path])

    def test_shared_motion_saves_both_areas_and_preserves_targets_and_p0(self):
        for area in ('coarse', 'storage'):
            with self.subTest(area=area):
                before = read_config(self.debug)
                values = {f['path']: get_value(before, f['path'], area) for f in area_fields(area)}
                values.update({'gantry_motion.max_yaw_rad_s': .08,
                               'gantry_motion.max_fb_rpm': 4.,
                               'gantry_motion.length_tolerance_pulses': 2.,
                               'gantry_motion.angle_tolerance_rad': .02,
                               'gantry_motion.yaw_pid.kp': .6,
                               'gantry_motion.yaw_pid.ki': .01,
                               'gantry_motion.length_pid.kp': .3,
                               'gantry_motion.timeout_s': 40.,
                               'angle_alignment.pid.kp': .2,
                               'base_servo.tolerance_px': 6.,
                               'gantry_servo.max_fb_rpm': 8.})
                saved = save_parameters(self.debug, before, values, area, self.paths)
                self.assertEqual(read_config(self.debug), saved)
                for group in SHARED_GROUPS:
                    self.assertEqual(saved['coarse'][group], saved['storage'][group])
                for other in ('coarse', 'storage'):
                    self.assertEqual(saved[other]['gantry_presets'], before[other]['gantry_presets'])
                    self.assertEqual(saved[other]['vision'], before[other]['vision'])
                    self.assertEqual(saved[other]['exit_motion'], before[other]['exit_motion'])
                    for path in self.paths[other]:
                        self.assertEqual(read_work_config(path)[other], saved[other])
                        self.assertTrue(set(SHARED_GROUPS).isdisjoint(yaml.safe_load(path.read_text())[other]))
                self.assertEqual(saved['raw']['gantry_motion'],saved['coarse']['gantry_motion'])
                self.assertEqual({k:v for k,v in saved['raw'].items() if k!='gantry_motion'},
                                 {k:v for k,v in before['raw'].items() if k!='gantry_motion'})
                for path in self.paths['raw']:
                    self.assertEqual(read_work_config(path)['raw'],saved['raw'])
                files = (self.debug, self.folder/'work_motion.yaml', *self.paths['coarse'], *self.paths['storage'])
                originals = {p: p.read_bytes() for p in files}
                values['gantry_motion.max_fb_rpm'] = 0
                with self.assertRaises(ValueError):
                    save_parameters(self.debug, saved, values, area, self.paths)
                for path, content in originals.items():
                    self.assertEqual(path.read_bytes(), content)

    def test_raw_shared_motion_editor_updates_three_regions_without_sharing_visual_pid(self):
        values={f['path']:get_value(self.config,f['path'],'raw') for f in area_fields('raw')}
        values['gantry_motion.yaw_pid.kp']=.75
        values['gantry_motion.length_pid.kd']=.01
        values['gantry_servo.max_fb_rpm']=3.
        saved=save_parameters(self.debug,self.config,values,'raw',self.paths)
        self.assertEqual(read_config(self.debug),saved)
        for area in ('raw','coarse','storage'):
            self.assertEqual(saved[area]['gantry_motion'],saved['raw']['gantry_motion'])
            for path in self.paths[area]:
                self.assertEqual(read_work_config(path)[area],saved[area])
        self.assertEqual(saved['coarse']['gantry_servo'],self.config['coarse']['gantry_servo'])

    def test_missing_destination_and_failed_replace_leave_all_files_unchanged(self):
        originals = {p: p.read_bytes() for p in (self.debug, *self.paths['raw'])}
        values = {f['path']: get_value(self.config, f['path'], 'raw') for f in area_fields('raw')}
        values['gantry_servo.max_fb_rpm'] = 4.
        with self.assertRaises(OSError):
            save_parameters(self.debug, self.config, values, 'raw', {'raw': [self.folder/'missing.yaml']})
        replace = os.replace

        def fail_copy(src, dst):
            if dst == self.paths['raw'][1]:
                raise OSError('installed copy failed')
            replace(src, dst)

        with patch('coarse_debug.persistence.os.replace', side_effect=fail_copy):
            with self.assertRaises(OSError):
                save_parameters(self.debug, self.config, values, 'raw', self.paths)
        for path, content in originals.items():
            self.assertEqual(path.read_bytes(), content)
        self.assertFalse(list(self.folder.glob('*.save.*')))

    def test_shared_camera_and_lease_settings_stay_compatible_with_formal_executor(self):
        values = {f['path']: get_value(self.config, f['path']) for f in area_fields('coarse')}
        values['vision.max_frame_age_s'] = .4
        values['execution.lease_timeout_s'] = 2
        saved = save_parameters(self.debug, self.config, values, 'coarse', self.paths)
        self.assertEqual(saved['storage']['vision']['max_frame_age_s'], .4)
        self.assertEqual(saved['raw']['vision'], self.config['raw']['vision'])
        for area, paths in self.paths.items():
            self.assertEqual(saved[area]['execution']['lease_timeout_s'], 2.)
            for path in paths:
                self.assertEqual(read_work_config(path)[area], saved[area])

    def test_invalid_input_never_changes_formal_configuration(self):
        originals = {p: p.read_bytes() for p in (self.debug, *self.paths['coarse'])}
        values = {f['path']: get_value(self.config, f['path']) for f in area_fields('coarse')}
        values['gantry_servo.max_fb_rpm'] = None
        with self.assertRaises(ValueError):
            save_parameters(self.debug, self.config, values, 'coarse', self.paths)
        for path, content in originals.items():
            self.assertEqual(path.read_bytes(), content)

    def test_deleted_command_cannot_be_saved_to_debug_or_formal_configuration(self):
        files = (self.debug, *(path for paths in self.paths.values() for path in paths))
        originals = {path: path.read_bytes() for path in files}
        for area, command in (('raw', 'pick_cmd'), ('coarse', 'place_cmd'), ('storage', 'place_cmd')):
            values = {f['path']: get_value(self.config, f['path'], area) for f in area_fields(area)}
            values[f'arm.{command}'] = 8
            with self.subTest(area=area), self.assertRaises(ValueError):
                save_parameters(self.debug, self.config, values, area, self.paths)
        for path, content in originals.items():
            self.assertEqual(path.read_bytes(), content)

    def test_failed_area_write_rolls_back_shared_motion_too(self):
        files = (self.debug, self.folder/'work_motion.yaml', *self.paths['coarse'], *self.paths['storage'])
        originals = {path: path.read_bytes() for path in files}
        values = {f['path']: get_value(self.config, f['path']) for f in area_fields('coarse')}
        values['gantry_servo.u_pid.kp'] *= .5
        replace = os.replace

        def fail_storage(source, destination):
            if destination == self.paths['storage'][1]:
                raise OSError('storage installed copy failed')
            replace(source, destination)

        with patch('coarse_debug.persistence.os.replace', side_effect=fail_storage):
            with self.assertRaises(OSError):
                save_parameters(self.debug, self.config, values, 'coarse', self.paths)
        for path, content in originals.items():
            self.assertEqual(path.read_bytes(), content)
        self.assertEqual(read_config(self.debug), self.config)
