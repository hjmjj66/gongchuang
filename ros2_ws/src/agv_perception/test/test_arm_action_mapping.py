"""Check that production task requests use the v0.8 region-specific programs."""
from pathlib import Path
import unittest

import yaml

from agv_perception.coarse_control import CoarseTask, validate_coarse_config
from agv_perception.raw_control import RawTask, validate_raw_config
from agv_perception.storage_control import StorageTask, validate_storage_config
from agv_perception.work_config import read_work_config


ROOT = Path(__file__).resolve().parents[4]


class ActionMappingTests(unittest.TestCase):
    def test_deleted_command_rejected_in_area_configuration(self):
        for area, validate, command in (
                ('raw', validate_raw_config, 'pick_cmd'),
                ('coarse', validate_coarse_config, 'place_cmd'),
                ('storage', validate_storage_config, 'place_cmd')):
            config = read_work_config(Path(__file__).parents[1]/f'config/{area}.yaml')[area]
            config['arm'][command] = 8
            with self.subTest(area=area), self.assertRaisesRegex(ValueError, '8已删除'):
                validate(config)

    def test_region_programs_in_production_and_web_configuration(self):
        debug = yaml.safe_load((ROOT/'tools/coarse_debug/config/debug.yaml').read_text())
        cases = (
            ('raw', RawTask, {'ENTER_WORK_POSE': 3,
                             'RESTORE_WORK': 3, 'PICK': 4, 'STOW_ARM': 2}),
            ('coarse', CoarseTask, {'WORK_ENTRY': 3, 'PLACE': 6, 'PICK': 5,
                                   'WORK_EXIT': 3, 'STOW': 2}),
            ('storage', StorageTask, {'WORK_ENTRY': 3, 'PLACE': 6, 'SCAN_RESTORE_WORK': 3,
                                     'STACK': 7, 'STOW': 2}),
        )
        for area, cls, commands in cases:
            config = yaml.safe_load((Path(__file__).parents[1]/f'config/{area}.yaml').read_text())[area]
            if area in ('raw', 'coarse', 'storage'):
                self.assertEqual(debug[area]['arm'], config['arm'])
            if area == 'raw':
                task = cls(config, area, (1, 2, 3), 0.)
            elif area == 'coarse':
                task = cls(config, area, (1, 2, 3), (1, 2, 3), 0.)
            else:
                task = cls(config, area, (1, 2, 3), (1, 2, 3), 0., 'STORAGE_PLACE', {})
            for state, expected in commands.items():
                with self.subTest(area=area, state=state):
                    task._enter(state, 1.)
                    self.assertEqual(task.arm_request['cmd'], expected)
                    self.assertEqual(task.velocity, (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
