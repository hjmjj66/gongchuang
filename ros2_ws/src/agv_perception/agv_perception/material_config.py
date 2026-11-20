"""Shared configuration boundary for Python, ROS and the material web preview."""

from pathlib import Path
import math
from dataclasses import asdict

import yaml

from .ring_geometry import GeometryConfig

COLORS = ('red', 'yellow', 'blue', 'green', 'black', 'light_blue')
TOP_LABELS = ('red_top', 'yellow_top', 'blue_top', 'green_top', 'black_top', 'lightblue_top')
WHOLE_LABELS = ('red', 'yellow', 'blue', 'green', 'black', 'lightblue')
DEPLOY = '/root/material_deploy_v1'
JHB_DEPLOY = '/root/去年开源/gc_material_model_v218_20261116'
DEFAULT_PARAMETERS = {
    'use_jhb_model': False,
    'test_combine': False, 'combine.min_iou': 0.3,
    'jhb.model_path': JHB_DEPLOY + '/detector/material_cap_v218_384x640_713700d9dc72.bin',
    'jhb.color_path': JHB_DEPLOY + '/color/color_lab_v3_rda_core_d6721d60cb26.json',
    'jhb.work_scale': 4, 'jhb.confidence_threshold': 0.25, 'jhb.nms_threshold': 0.45,
    'jhb.cap_min_score': 0.7, 'jhb.material_min_score': 0.25,
    'jhb.border_px': 1.2, 'jhb.min_axis_ratio': 0.8, 'jhb.min_containment': 0.7,
    'jhb.white_balance': True, 'jhb.color_p_min': 0.9, 'jhb.color_d2_max': 23.172,
    **{'jhb.geometry.' + k: v for k, v in asdict(GeometryConfig(min_radius_ratio=0.35)).items()},
    'material_backend': 2, 'material_model': 1, 'target_color_id': 0, 'search_roi': [],
    'model_format': 'rdk_yolov8_bin', 'model_runtime_path': DEPLOY + '/rdk_model_zoo',
    'top.model_path': DEPLOY + '/export/upper_top/compiled_int8/upper_top_bayese_640x640_nv12.bin',
    'top.classes_path': DEPLOY + '/export/upper_top/classes.txt', 'top.class_labels': list(TOP_LABELS),
    'whole.model_path': DEPLOY + '/export/whole/compiled_int8/whole_bayese_640x640_nv12.bin',
    'whole.classes_path': DEPLOY + '/export/whole/classes.txt', 'whole.class_labels': list(WHOLE_LABELS),
    'confidence_threshold': 0.5, 'nms_threshold': 0.7,
    'top.refine_circle': True, 'top.allow_bbox_fallback': True,
    'top.roi_padding_ratio': 0.08, 'top.minimum_roi_size_px': 20, 'border_tolerance_px': 1.0,
    'top_circle.blur_kernel': 5, 'top_circle.blur_sigma': 1.2,
    'top_circle.hough_dp': 1.0, 'top_circle.hough_min_distance_px': 10.0,
    'top_circle.hough_param1': 100.0, 'top_circle.hough_param2': 25.0,
    'top_circle.min_radius_ratio': 0.35, 'top_circle.max_radius_ratio': 0.55,
    'top_circle.max_center_offset_ratio': 0.2, 'top_circle.min_edge_support': 0.5,
    'top_circle.edge_radial_tolerance_px': 2,
    'top_circle.duplicate_center_distance_px': 2.0,
    'top_circle.duplicate_radius_difference_px': 2.0, 'top_circle.ambiguity_margin': 0.03,
    'hsv.red_ranges': [0,80,80,10,255,255,174,135,58,179,237,198],
    'hsv.yellow_ranges': [12,117,114,34,224,209],
    'hsv.blue_ranges': [81,94,10,129,255,132],
    'hsv.green_ranges': [41,117,38,70,201,183],
    'hsv.black_ranges': [0,0,17,135,79,40],
    'hsv.light_blue_ranges': [72,71,63,179,163,209],
    'hsv.min_contour_area_px2': 100.0, 'hsv.max_contour_area_ratio': 0.0,
    'hsv.morphology_kernel': 3, 'hsv.open_iterations': 0, 'hsv.close_iterations': 0,
    'hsv.color_overlap_threshold': 0.6,
    'debug_mode': 'detect', 'input_image': '', 'manual_roi': [], 'manual_color_id': 0,
    'show_preview': False, 'output_dir': '', 'save_every_n_frames': 0,
    'camera_device': '/dev/v4l/by-id/usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0',
    'camera_width': 1280, 'camera_height': 720, 'camera_fps': 30.0, 'camera_buffer_size': 4,
    'camera_dynamic_framerate': False, 'process_rate_hz': 30.0, 'opencv_threads': 2,
    'max_frame_age_s': 0.5, 'camera_start_timeout_s': 10.0,
}


def parse_material_config(parameters):
    if not isinstance(parameters, dict):
        raise ValueError('material parameters must be a mapping')
    unknown = parameters.keys() - DEFAULT_PARAMETERS.keys()
    if unknown:
        raise ValueError(f'未知物料参数：{sorted(unknown)}')
    p = {k: list(v) if isinstance(v, list) else v for k, v in DEFAULT_PARAMETERS.items()}
    p.update(parameters)
    for name in ('material_backend', 'material_model'):
        _integer(p, name, 1, 2)
    _integer(p, 'target_color_id', 0, 6)
    _integer(p, 'manual_color_id', 0, 6)
    if p['debug_mode'] not in ('detect', 'geometry_only'):
        raise ValueError('debug_mode must be detect or geometry_only')
    for name in ('search_roi', 'manual_roi'):
        _roi(p[name], name)
    for name in ('input_image', 'output_dir', 'camera_device'):
        if not isinstance(p[name], str):
            raise ValueError(f'{name} must be a string')
    for name in ('show_preview', 'camera_dynamic_framerate', 'use_jhb_model', 'test_combine'):
        _boolean(p, name)
    if p['test_combine'] and p['use_jhb_model']:
        raise ValueError('test_combine 与 use_jhb_model 不能同时启用')
    if p['test_combine']:
        _number(p, 'combine.min_iou', 0, 1, exclusive=True)
    for name in ('camera_width', 'camera_height', 'camera_buffer_size', 'opencv_threads'):
        _integer(p, name, 1)
    _integer(p, 'save_every_n_frames', 0)
    for name in ('camera_fps', 'process_rate_hz', 'max_frame_age_s', 'camera_start_timeout_s'):
        _number(p, name, 0, exclusive=True)
    _number(p, 'border_tolerance_px', 0)
    manual = p['debug_mode'] == 'geometry_only'
    if manual:
        if not p['input_image'] or not p['manual_roi'] or not p['manual_color_id']:
            raise ValueError('geometry_only 需要 input_image、manual_roi 和 manual_color_id=1～6')
    elif p['manual_roi']:
        raise ValueError('manual_roi 仅用于 geometry_only')
    jhb = p['use_jhb_model'] or p['test_combine']
    if jhb and not manual:
        for name in (('jhb.model_path',) if p['test_combine'] else ('jhb.model_path', 'jhb.color_path')):
            if not isinstance(p[name], str) or not p[name].strip():
                raise ValueError(f'{name} must be a nonempty path')
        if Path(p['jhb.model_path']).suffix not in ('.bin', '.onnx'):
            raise ValueError('jhb.model_path must be .bin (X5) or .onnx (OpenCV)')
        _integer(p, 'jhb.work_scale', 1)
        for name in ('confidence_threshold', 'nms_threshold', 'cap_min_score',
                     'material_min_score', 'min_axis_ratio', 'min_containment'):
            _number(p, 'jhb.' + name, 0, 1)
        _number(p, 'jhb.border_px', 0)
        if p['use_jhb_model']:
            _number(p, 'jhb.color_p_min', 0, 1)
            _number(p, 'jhb.color_d2_max', 0, exclusive=True)
            _boolean(p, 'jhb.white_balance')
        _boolean(p, 'top.refine_circle')
        _boolean(p, 'top.allow_bbox_fallback')
        _number(p, 'top.roi_padding_ratio', 0)
        _integer(p, 'top.minimum_roi_size_px', 3)
        GeometryConfig(**{k.removeprefix('jhb.geometry.'): v for k, v in p.items()
                          if k.startswith('jhb.geometry.')})
    if (p['test_combine'] or (p['material_backend'] == 2 and not jhb)) and not manual:
        branch = 'top' if p['material_model'] == 1 else 'whole'
        expected = TOP_LABELS if branch == 'top' else WHOLE_LABELS
        labels = p[branch + '.class_labels']
        if not isinstance(labels, (list, tuple)) or tuple(labels) != expected:
            raise ValueError(f'{branch}.class_labels 必须按部署顺序为 {expected}')
        for name in ('model_format', 'model_runtime_path', branch + '.model_path', branch + '.classes_path'):
            if not isinstance(p[name], str):
                raise ValueError(f'{name} must be a string')
        if p['model_format'] != 'rdk_yolov8_bin':
            raise ValueError('model_format must be rdk_yolov8_bin')
        for name in ('confidence_threshold', 'nms_threshold'):
            _number(p, name, 0, 1)
    if manual or (not jhb and p['material_backend'] == 2 and p['material_model'] == 1):
        _boolean(p, 'top.refine_circle')
        _boolean(p, 'top.allow_bbox_fallback')
        if p['top.refine_circle']:
            _number(p, 'top.roi_padding_ratio', 0)
            _integer(p, 'top.minimum_roi_size_px', 1)
            _odd(p, 'top_circle.blur_kernel')
            for name in ('blur_sigma', 'max_center_offset_ratio', 'duplicate_center_distance_px',
                         'duplicate_radius_difference_px', 'ambiguity_margin'):
                _number(p, 'top_circle.' + name, 0)
            for name in ('hough_dp', 'hough_min_distance_px', 'hough_param1', 'hough_param2',
                         'min_radius_ratio', 'max_radius_ratio'):
                _number(p, 'top_circle.' + name, 0, exclusive=True)
            if p['top_circle.hough_dp'] < 1:
                raise ValueError('top_circle.hough_dp must be >= 1')
            if p['top_circle.max_radius_ratio'] < p['top_circle.min_radius_ratio']:
                raise ValueError('top_circle radius ratios are reversed')
            _number(p, 'top_circle.min_edge_support', 0, 1)
            _integer(p, 'top_circle.edge_radial_tolerance_px', 0)
    if p['material_backend'] == 1 and not manual and not jhb:
        for color in COLORS:
            name = f'hsv.{color}_ranges'
            values = p[name]
            if not isinstance(values, (list, tuple)) or len(values) % 6:
                raise ValueError(f'{name} must contain groups of six integers')
            for i in range(0, len(values), 6):
                lo, hi = values[i:i+3], values[i+3:i+6]
                if any(type(v) is not int for v in (*lo, *hi)) or any(
                        not 0 <= a <= b <= limit for a, b, limit in zip(lo, hi, (179,255,255))):
                    raise ValueError(f'{name} has invalid HSV limits')
        _number(p, 'hsv.min_contour_area_px2', 0, exclusive=True)
        _number(p, 'hsv.max_contour_area_ratio', 0, 1)
        _odd(p, 'hsv.morphology_kernel')
        for name in ('hsv.open_iterations', 'hsv.close_iterations'):
            _integer(p, name, 0)
        _number(p, 'hsv.color_overlap_threshold', 0, 1, exclusive=True)
    if not p['input_image'] and not p['camera_device'].strip():
        raise ValueError('camera_device must not be empty')
    return p


def material_model_info(p):
    if p['test_combine'] and p['debug_mode'] != 'geometry_only':
        branch = 'top' if p['material_model'] == 1 else 'whole'
        return {'backend': 'test_combine', 'model_kind': 'test_combine',
                'model_path': p['jhb.model_path'], 'color_model_path': p[branch + '.model_path'],
                'class_labels': p[branch + '.class_labels'],
                'confidence_threshold': p['confidence_threshold'],
                'refine_circle': p['top.refine_circle']}
    if p['use_jhb_model'] and p['debug_mode'] != 'geometry_only':
        return {'backend': 'jhb', 'model_kind': 'jhb', 'model_path': p['jhb.model_path'],
                'class_labels': ['material', 'cap_top'],
                'confidence_threshold': p['jhb.confidence_threshold'],
                'refine_circle': p['top.refine_circle']}
    branch = 'top' if p['material_model'] == 1 else 'whole'
    manual = p['debug_mode'] == 'geometry_only'
    yolo = p['material_backend'] == 2 and not manual
    return {'backend': 'manual' if manual else 'yolo' if yolo else 'hsv',
            'model_kind': 'top' if manual else branch if yolo else None,
            'model_path': p[branch + '.model_path'] if yolo else None,
            'class_labels': p[branch + '.class_labels'] if yolo else [],
            'confidence_threshold': p['confidence_threshold'] if yolo else None,
            'refine_circle': p['top.refine_circle'] if manual or yolo and branch == 'top' else False}


def read_material_config(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('material_debug_node'), dict):
        raise ValueError('配置须包含 material_debug_node.ros__parameters')
    return parse_material_config(data['material_debug_node'].get('ros__parameters'))


def _integer(p, name, minimum, maximum=None):
    value = p[name]
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f'{name} must be an integer in [{minimum}, {maximum}]')


def _number(p, name, minimum, maximum=None, exclusive=False):
    value = p[name]
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (value <= minimum if exclusive else value < minimum)
            or (maximum is not None and value > maximum)):
        raise ValueError(f'{name} has an invalid numeric value')


def _boolean(p, name):
    if type(p[name]) is not bool:
        raise ValueError(f'{name} must be boolean')


def _odd(p, name):
    _integer(p, name, 1)
    if p[name] % 2 != 1:
        raise ValueError(f'{name} must be odd')


def _roi(value, name):
    if not isinstance(value, (list, tuple)) or (value and (
            len(value) != 4 or any(type(v) is not int for v in value)
            or value[2] <= value[0] or value[3] <= value[1] or min(value) < 0)):
        raise ValueError(f'{name} must be [] or integer [x1,y1,x2,y2] with positive size')
