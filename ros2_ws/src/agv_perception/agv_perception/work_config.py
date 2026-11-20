"""Load area settings and the single shared source of work motion parameters."""

from copy import deepcopy
from pathlib import Path

import yaml


SHARED_GROUPS = ('angle_alignment', 'base_servo', 'gantry_servo', 'gantry_motion')


def read_work_config(path):
    path = Path(path)
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if 'motion_config' in config:
        motion_path = (path.parent / config['motion_config']).resolve()
        config['motion_config'] = str(motion_path)
        motion = yaml.safe_load(motion_path.read_text(encoding='utf-8'))
        for area in ('raw', 'coarse', 'storage'):
            if area in config:
                groups = ('gantry_motion',) if area == 'raw' else SHARED_GROUPS
                config[area].update({group: deepcopy(motion[group]) for group in groups})
    return config


def area_settings(config, area):
    settings = deepcopy(config[area])
    if 'motion_config' in config:
        for group in (('gantry_motion',) if area == 'raw' else SHARED_GROUPS):
            settings.pop(group)
    return settings
