"""Three camera aim points, captured from displayed frames and synced to task YAML."""

from collections import OrderedDict
from copy import deepcopy
import math
from pathlib import Path
import uuid

import yaml

from .config import validate
from .persistence import replace_value, write_configs
from agv_perception.gantry_presets import DEFAULTS, validate_presets
from agv_perception.work_config import read_work_config


TYPES = {
    'raw': dict(label='原料区取料', area='raw', visit=1, kind='material',
                destinations=(('raw', 'target_uv_px'),)),
    'ring': dict(label='粗加工 / 暂存第一次圆环', area='coarse', visit=1, kind='ring',
                 destinations=(('coarse', 'target_uv_px'), ('storage', 'target_uv_px'))),
    'stack': dict(label='暂存第二次物料', area='storage', visit=2, kind='material',
                  destinations=(('storage', 'stack_target_uv_px'),)),
    'shared_pose': dict(label='粗加工 / 暂存区共用机构初值', area='coarse', visit=1, kind='ring', preset=True,
                        destinations=(('coarse', 'gantry_presets'), ('storage', 'gantry_presets'))),
}


class CalibrationFrames:
    """Remember the exact inference packet attached to each HTTP image response."""
    def __init__(self):
        self.frames = OrderedDict()

    def register(self, obs, category, selection_id):
        token = uuid.uuid4().hex
        self.frames[token] = (obs, category, selection_id)
        while len(self.frames) > 64:
            self.frames.popitem(last=False)
        return dict(token=token, category=category, selection_id=selection_id, kind=obs.kind,
                    sequence=obs.sequence, image_size=obs.image_size,
                    rings=obs.rings, materials=obs.materials)

    def capture(self, settings, category, selection_id, now, max_age):
        record = self.frames.get(settings.get('frame_token', ''))
        if not record or record[1:] != (category, selection_id):
            raise ValueError('显示画面已切换，请等待当前标定类型的新画面后再采点')
        obs = record[0]
        if not 0 <= now-obs.captured_at <= max_age:
            raise ValueError('采点画面已过期，请等待实时画面后重试')
        spec = TYPES[category]
        target = settings.get('target')
        maximum = 3 if spec['kind'] == 'ring' else 6
        if obs.kind != spec['kind'] or type(target) is not int or not 1 <= target <= maximum:
            raise ValueError('标定目标类型或编号无效')
        points = ([obs.rings[target]] if target in obs.rings else []) if obs.kind == 'ring' else [
            point for color, point in obs.materials if color == target]
        if len(points) != 1:
            raise ValueError('所选目标不可见或存在多个同色目标，请调整画面后重新采点')
        point = points[0]
        if (len(point) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) or
                                  not 0 <= v < size for v, size in zip(point, obs.image_size))):
            raise ValueError('识别坐标超出原图或无效')
        return dict(id=uuid.uuid4().hex, category=category, point=list(point),
                    sequence=obs.sequence, image_size=obs.image_size, target=target)


def _replace_point(text, area, key, point, section='vision'):
    keys = (area, section, key) if section else (area, key)
    return replace_value(text, keys, point)


def save_calibration(debug_path, config, category, point, production_paths, *, ring=None):
    if category not in TYPES:
        raise ValueError('标定类型或坐标无效')
    preset = TYPES[category].get('preset', False)
    if preset:
        if type(ring) is not int or ring not in (1, 3):
            raise ValueError('请选择1号或3号位置')
        candidate_pose = deepcopy(DEFAULTS)
        candidate_pose['positions'][str(ring)] = point
        if point is None:
            raise ValueError('缺少机构实际位置')
        validate_presets(candidate_pose)
        shared_positions = deepcopy(config['coarse'].get('gantry_presets', DEFAULTS)['positions'])
        shared_positions[str(ring)] = point
    elif (not isinstance(point, list) or len(point) != 2 or
          any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in point)):
        raise ValueError('标定类型或坐标无效')
    # Resolve symlinks before replacement, so a symlink-install remains linked to source.
    debug_path = Path(debug_path).resolve()
    edits = OrderedDict()
    for area, key in TYPES[category]['destinations']:
        edits.setdefault(debug_path, []).append((area, key))
        destinations = production_paths[area]
        if not isinstance(destinations, (list, tuple)):
            destinations = (destinations,)
        for destination in destinations:
            fields = edits.setdefault(Path(destination).resolve(), [])
            if (area, key) not in fields:
                fields.append((area, key))
    changes = {}
    try:
        for path, fields in edits.items():
            original = path.read_text(encoding='utf-8')
            text = original
            before = yaml.safe_load(original)
            # Preserve the old second-visit point when upgrading a legacy shared point.
            if not preset and 'storage' in before and 'stack_target_uv_px' not in before['storage']['vision']:
                legacy = (config['storage']['vision'].get('stack_target_uv_px', config['storage']['vision']['target_uv_px'])
                          if path == debug_path else before['storage']['vision']['target_uv_px'])
                text = _replace_point(text, 'storage', 'stack_target_uv_px', legacy)
            for area, key in fields:
                value = point
                if preset:
                    value = deepcopy(before[area].get('gantry_presets', DEFAULTS))
                    value['positions'] = shared_positions
                text = _replace_point(text, area, key, value, section=None if preset else 'vision')
            document = yaml.safe_load(text)
            if path == debug_path:
                candidate = read_work_config(path)
                for area, key in fields:
                    if preset:
                        candidate[area][key] = deepcopy(document[area][key])
                    else:
                        candidate[area]['vision'][key] = document[area]['vision'][key]
                candidate['storage']['vision'].setdefault('stack_target_uv_px',
                    deepcopy(config['storage']['vision'].get('stack_target_uv_px',
                                                           config['storage']['vision']['target_uv_px'])))
                validate(candidate)
            # Check only the changed values: other formal PID parameters may still be null.
            for area, key in fields:
                written = (document[area][key]['positions'] if preset else document[area]['vision'][key])
                if written != (shared_positions if preset else point):
                    raise ValueError(f'标定写入校验失败：{path}')
            changes[path] = (original, text)
        write_configs(changes)
    except yaml.YAMLError as error:
        raise ValueError(f'配置YAML格式错误，未保存标定点：{error}') from error
    return candidate
