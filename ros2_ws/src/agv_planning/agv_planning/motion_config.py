"""Navigation parameters shared by the planner and route tuning page."""

import json
import math
import os
from pathlib import Path
import tempfile

import yaml


MOTION_PARAMETERS = {
    'translation_pid_kp': ('_translation_kp', 2.0),
    'translation_pid_ki': ('_translation_ki', 0.0),
    'translation_pid_kd': ('_translation_kd', 0.1),
    'translation_max_vx_mps': ('_vx_max', 0.7),
    'translation_max_vy_mps': ('_vy_max', 0.4),
    'translation_arrival_tolerance_mm': ('_translation_tolerance_mm', 3.0),
    'angle_pid_kp': ('_angle_kp', 2.0),
    'angle_pid_ki': ('_angle_ki', 0.0),
    'angle_pid_kd': ('_angle_kd', 0.1),
    'angle_max_speed_deg_s': ('_w_max', 120.0),
    'turn_arrival_tolerance_deg': ('_turn_tolerance_deg', 0.2),
}


def validate_motion_parameters(values, shortest_edge_mm):
    if set(values) != set(MOTION_PARAMETERS):
        raise ValueError('导航参数字段不完整或包含未知字段，请刷新页面')
    for name, value in values.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{name} 必须为有限数值')
        positive = name.endswith('_kp') or 'max_' in name
        if value < 0 or (positive and value == 0):
            raise ValueError(f'{name} 必须大于0' if positive else f'{name} 不能为负数')
    if values['translation_arrival_tolerance_mm'] >= shortest_edge_mm:
        raise ValueError('位置到达容差必须小于最短路段长度')
    if values['turn_arrival_tolerance_deg'] >= 90:
        raise ValueError('角度到达容差必须小于90°')


def save_motion_parameters(path, values):
    path = Path(path).resolve()
    text = path.read_text(encoding='utf-8')
    try:
        document = yaml.compose(text)
        root = dict((key.value, value) for key, value in document.value)
        node = dict((key.value, value) for key, value in root['route_planner_node'].value)['ros__parameters']
        fields = {key.value: value for key, value in node.value}
    except (yaml.YAMLError, AttributeError, KeyError, TypeError, ValueError) as exc:
        raise ValueError('规划配置缺少 route_planner_node.ros__parameters 映射') from exc
    # Edit scalar spans so field notes and unrelated settings survive saving.
    if node.flow_style:
        merged = yaml.safe_load(text[node.start_mark.index:node.end_mark.index])
        merged.update(values)
        edits = [(node.start_mark.index, node.end_mark.index, json.dumps(merged, ensure_ascii=False))]
    else:
        edits = [(fields[name].start_mark.index, fields[name].end_mark.index, str(float(value)))
                 for name, value in values.items() if name in fields]
        missing = {name: value for name, value in values.items() if name not in fields}
        if missing:
            addition = ''.join(f'{name}: {float(value)}\n' + ' '*node.start_mark.column
                               for name, value in missing.items())
            edits.append((node.start_mark.index, node.start_mark.index, addition))
    for start, end, replacement in sorted(edits, reverse=True):
        text = text[:start] + replacement + text[end:]
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                         prefix=path.name+'.save.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
