"""Editable motion parameters validated once at the file/HTTP boundary."""

from copy import deepcopy
import math
from pathlib import Path

import yaml

from .persistence import replace_parameters, write_configs
from agv_perception.work_config import SHARED_GROUPS, area_settings, read_work_config


FIXED_FIELDS = {
    'execution.control_rate_hz': '控制频率 Hz（固定）',
    'angle_alignment.target_slope': '水平目标斜率（固定）',
    'gantry_servo.qualified_frames': '机构对准连续达标帧数（固定）',
}


def pid_label(section, axis, term):
    if axis == 'pid':
        name, source, output = '底盘水平纠偏', '斜率', 'rad/s'
    elif axis in ('x_pid', 'y_pid'):
        name, source, output = axis[0]+'里程轴', 'm', 'm/s'
    elif axis == 'length_pid':
        name, source, output = '机构伸缩', '脉冲', 'RPM'
    elif axis == 'yaw_pid':
        name, source, output = '机构旋转', 'rad' if section == 'gantry_motion' else 'px', 'rad/s'
    elif axis == 'fb_pid':
        name, source, output = '机构伸缩', 'px', 'RPM'
    else:
        name, source = ('水平像素轴' if axis == 'u_pid' else '竖直像素轴'), 'px'
        output = 'm/s' if section == 'base_servo' else ('rad/s' if axis == 'u_pid' else 'RPM')
    return f'{name} {term.title()}（{source} → {output}）'


def coarse_fields(area='coarse'):
    p = read_work_config(Path(__file__).resolve().parents[1]/'config/debug.yaml')[area]
    labels = {
        'work_pose_cmd': '高位工作位动作号', 'place_cmd': '真实放料动作号',
        'stack_cmd': '真实码垛动作号', 'scan_settle_frames': '扫描每色连续确认帧数',
        'pick_cmd': '真实取料动作号', 'stow_cmd': '行驶收纳动作号',
        'lower_speed_rpm': '实际下降转速 RPM', 'lower_duration_s': '每件实际下降时长 s',
        'max_yaw_rad_s': '机构旋转限速 rad/s', 'max_fb_rpm': '机构伸缩限速 RPM',
        'small_yaw_rad_s': '对准旋转小输出门槛 rad/s', 'small_fb_rpm': '对准伸缩小输出门槛 RPM',
        'length_tolerance_pulses': '伸缩到位容差 脉冲', 'angle_tolerance_rad': '旋转到位容差 rad',
        'position_tolerance_m': '返回P0位置容差 m', 'kp': '返回P0比例增益',
        'yaw_rad_s': '搜索旋转速度 rad/s', 'duration_s': '单件略转搜索上限 s',
        'tolerance_px': '中心欧氏容差 px', 'max_vx_mps': '底盘vx限速 m/s', 'max_vy_mps': '底盘vy限速 m/s',
        'timeout_s': '机构到位时限 s', 'max_wz_rad_s': '底盘纠偏限速 rad/s',
        'min_horizontal_span_px': '拟合最小横向跨度 px', 'slope_tolerance': '水平斜率容差',
        'settle_frames': '连续确认帧数', 'stop_speed_mps': '底盘停车阈值 m/s',
        'max_frame_age_s': '图像有效期 s', 'camera_start_timeout_s': '相机启动时限 s',
        'zero_settle_time_s': '停止交接等待 s', 'feedback_timeout_s': '实际反馈有效期 s',
        'arm_request_timeout_s': '机构服务受理时限 s', 'lease_timeout_s': '控制许可有效期 s',
    }
    result = [dict(path='vision.target_uv_px', label='工作位底盘/机构共用准心 [u,v]', point=True)]
    if area == 'storage':
        result.append(dict(path='vision.stack_target_uv_px', label='暂存第二次物料码垛准心 [u,v]', point=True))
    for section, values in p.items():
        for name, value in values.items():
            if section == 'gantry_presets':
                continue
            if isinstance(value, (str, list)) or f'{section}.{name}' in FIXED_FIELDS:
                continue
            for term, number in (value.items() if isinstance(value, dict) else [(None, value)]):
                result.append(dict(path=f'{section}.{name}'+(f'.{term}' if term else ''),
                                   label=pid_label(section, name, term) if term else
                                         '水平纠偏时限 s（最多3秒）' if section == 'angle_alignment' and name == 'timeout_s' else
                                         '底盘2号环PID最长时间 s' if section == 'base_servo' and name == 'timeout_s' else
                                         '机构圆环PID最长时间 s' if section == 'gantry_servo' and name == 'timeout_s' else
                                         '收纳后返回P0时限 s' if section == 'exit_motion' and name == 'timeout_s' else
                                         labels.get(name, name),
                                   minimum=0., positive=term not in ('ki', 'kd'),
                                   maximum=7 if section == 'arm' and name.endswith('_cmd') else
                                           3 if section == 'angle_alignment' and name == 'timeout_s' else None,
                                   integer=section == 'arm' or name in ('settle_frames', 'scan_settle_frames')))
    return result


FIELDS = coarse_fields()
STORAGE_FIELDS = coarse_fields('storage')


# Keep raw numeric fields in sync with the production schema; labels retain protocol units.
_RAW_LABELS = {
    'qualified_frames': '抓取连续达标帧数',
    'max_vx_mps': '底盘前后限速 m/s', 'max_vy_mps': '底盘左右限速 m/s',
    'position_tolerance_m': '底盘进退到位容差 m', 'stop_speed_mps': '底盘停车速度门槛 m/s',
    'kp': '底盘位置比例增益',
    'max_yaw_rad_s': '旋转上限 rad/s', 'max_fb_rpm': '伸缩上限 RPM',
    'small_yaw_rad_s': '抓取旋转小输出 rad/s', 'small_fb_rpm': '抓取伸缩小输出 RPM',
    'tolerance_px': '中心容差 px', 'distance_mm': '底盘工作方向前进 / 退回距离 mm',
    'max_frame_age_s': '图像有效期 s', 'camera_start_timeout_s': '新姿态图像启动时限 s',
    'timeout_s': '运动到位时限 s', 'zero_settle_time_s': '机构停止交接等待 s',
    'length_tolerance_pulses': '伸缩到位容差 脉冲', 'angle_tolerance_rad': '旋转到位容差 rad',
    'feedback_timeout_s': '反馈有效期 s', 'arm_request_timeout_s': '固定动作服务受理时限 s',
    'lease_timeout_s': '执行许可有效期 s',
    'work_pose_cmd': '工作位动作号', 'pick_cmd': '真实抓取动作号', 'stow_cmd': '收纳动作号',
    'enabled': '启用卡尔曼与短时预测', 'measurement_std_px': '中心测量标准差 px',
    'acceleration_std_px_s2': '相对像素加速度标准差 px/s²',
    'initial_velocity_std_px_s': '初始速度标准差 px/s',
    'extra_delay_s': '额外相机/执行延迟 s（待实测）', 'max_horizon_s': '最长短时预测 s',
}

def raw_fields():
    p = read_work_config(Path(__file__).resolve().parents[1]/'config/debug.yaml')['raw']
    result = [dict(path='vision.target_uv_px', label='机构跟随/抓取准心 [u,v]；空为图像中心', point=True)]
    for section, values in p.items():
        for name, value in values.items():
            path = f'{section}.{name}'
            if path in FIXED_FIELDS or isinstance(value, (list, str)):
                continue
            if type(value) is bool:
                result.append(dict(path=path, label=_RAW_LABELS.get(name, name), boolean=True))
                continue
            terms = value.items() if isinstance(value, dict) else [(None, value)]
            for term, number in terms:
                result.append(dict(path=path+(f'.{term}' if term else ''),
                                   label=pid_label(section, name, term) if term else _RAW_LABELS.get(name, name),
                                   minimum=0., positive=term not in ('ki', 'kd') and name != 'extra_delay_s',
                                   maximum=7 if section == 'arm' else None,
                                   integer=section == 'arm' or name in ('qualified_frames', 'settle_frames', 'min_samples', 'max_samples')))
    return result

RAW_FIELDS = raw_fields()


def area_fields(area):
    return {'raw': RAW_FIELDS, 'storage': STORAGE_FIELDS, 'coarse': FIELDS}[area]


def fixed_parameters(config, area):
    return [dict(path=path, label=label, value=get_value(config, path, area))
            for path, label in FIXED_FIELDS.items()
            if path.split('.')[1] in config[area].get(path.split('.')[0], {})]


def parameter_groups(area):
    name = {'raw': '原料区', 'coarse': '粗加工区', 'storage': '暂存区'}[area]
    groups = {
        'vision': dict(label=f'{name} · 图像与准心', hint='准心为原图像素[u,v]，留空使用图像中心；可手填或在标定模式采点。圆环采点会同时同步两区准心，手填仅保存当前区域；粗加工和暂存共用图像有效期及相机启动时限。' if area != 'raw' else '准心为原图像素[u,v]，留空使用图像中心；原料机构跟随及抓取共用，可手填或在标定模式采点。'),
        'base_motion': dict(label='原料区 · 底盘进退', hint='沿到站车头前进，三件抓完收纳后退回。比赛开启离区合并时由导航吸收偏移，网页单独作业实际退回。距离单位mm。'),
        'exit_motion': dict(label=f'{name} · 底盘返回P0', hint='收纳后按二维里程返回P0；仅影响当前区域，与导航参数分别保存。'),
        'arm': dict(label=f'{name} · 机构动作号', hint='工作位、收纳及真实取放的下位机程序号，须与电控协议一致；1秒模拟仅替代取放，追踪中的取放按钮始终真实执行。固定程序自身的速度由下位机设置。'),
        'execution': dict(label=f'{name} · 执行与反馈时限', hint='控制频率固定50 Hz；反馈有效期、机构受理和停车交接等待用于作业及追踪。控制许可有效期在三个区域同步，其余设置按当前区域保存。'),
        'ring_search': dict(label=f'{name} · 机构寻找与扫描', hint='作业时在工作姿态附近略转搜索；旋转速度为rad/s、时长为秒。暂存第二次另有每色连续确认帧数；手动连续追踪不搜索。'),
        'grasp_gate': dict(label='原料区 · 机构对准 / 抓取门槛', hint='中心误差与旋转、伸缩小输出须连续达到设定帧数才抓取，默认4帧；只累计真实新帧。原料连续追踪也使用这些门槛显示对准状态。两轴PID及限速在机构跟随组。'),
        'motion_prediction': dict(label='原料区 · 卡尔曼与短时延迟补偿', hint='原料作业及连续追踪共用；估计像素位置和速度，补偿短时延迟。抓取达标仍按真实新帧判断。'),
    }
    groups['gantry_motion'] = dict(label='三区域共用 · 机构前往标定点 / 粗加工取回', hint='三个区域共用实际角度与伸缩脉冲位置PID、限速、到位容差和超时。任一区域保存均同步。')
    if area == 'raw':
        groups['gantry_servo'] = dict(label='原料区 · 机构旋转 / 伸缩跟随', hint='原料逐件跟随及连续追踪共用；像素误差 → 旋转rad/s、伸缩RPM。对准容差和小输出门槛在抓取门槛组。')
    else:
        for group, label, hint in (
            ('angle_alignment', '底盘水平纠偏', '圆环排列斜率 → 底盘角速度rad/s；共用PID、限速、斜率容差、确认帧数及最多3秒时限。'),
            ('base_servo', '底盘对准2号环', '像素误差 → 底盘vx/vy（m/s）；共用PID、限速、像素容差、停稳阈值、确认帧数及对准时限。'),
            ('gantry_servo', '机构视觉对环 / 对物料', '粗加工对环、暂存首次对环及第二次码垛共用；像素误差 → 旋转rad/s、伸缩RPM。包括容差、小输出门槛和圆环作业时限；连续追踪及暂存第二次不使用该时限。准心仍按作业设置。'),
        ):
            groups[group] = dict(label=f'粗加工 + 暂存共用 · {label}', hint=hint+' 在任一区域保存均影响两个区域。')
    return groups


def get_value(config, path, area='coarse'):
    value = config[area]
    for key in path.split('.'):
        value = value[key]
    return value


def validate(config):
    try:
        for area in ('coarse', 'storage', 'raw'):
            validate_area(config, area)
        d = config['debug']
        if (not isinstance(d['host'], str) or type(d['port']) is not int or not 1 <= d['port'] <= 65535 or
                type(d['preview_width']) is not int or d['preview_width'] < 1 or
                type(d['browser_timeout_s']) not in (int, float) or
                not math.isfinite(d['browser_timeout_s']) or d['browser_timeout_s'] <= 0):
            raise ValueError('网页端口、预览宽度或心跳配置非法')
        if not Path(config['storage']['vision']['material_config']).is_file():
            raise ValueError('物料配置文件不存在')
        if config['coarse']['vision']['ring_config'] != config['storage']['vision']['ring_config']:
            raise ValueError('两个区域须使用同一圆环识别配置')
        if config['raw']['vision']['material_config'] != config['storage']['vision']['material_config']:
            raise ValueError('原料与暂存须使用同一物料识别配置')
    except (KeyError, TypeError) as error:
        raise ValueError(f'调试配置格式不完整：{error}') from error


def validate_area(config, area):
    for item in area_fields(area):
        name, value = item['path'], get_value(config, item['path'], area)
        if item.get('boolean'):
            if type(value) is not bool:
                raise ValueError(f'{name} 必须为布尔值')
            continue
        if item.get('point'):
            if (not isinstance(value, list) or len(value) not in (0, 2) or
                    any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in value)):
                raise ValueError(f'{name} 必须为空或非负的 [u,v]')
            continue
        if (type(value) not in (int, float) or not math.isfinite(value) or
                (value <= 0 if item['positive'] else value < 0) or
                (item['maximum'] is not None and value > item['maximum']) or
                (item['integer'] and type(value) is not int)):
            raise ValueError(f'参数非法：{name}')
    p = config[area]
    if area == 'coarse':
        from agv_perception.coarse_control import validate_coarse_config
        validate_coarse_config(p)
        if p['execution']['control_rate_hz'] != 50:
            raise ValueError('粗加工控制频率固定50 Hz')
        return
    if area == 'raw':
        from agv_perception.raw_control import validate_raw_config
        validate_raw_config(p)
        if p['execution']['control_rate_hz'] != 50:
            raise ValueError('原料控制频率固定50 Hz')
        return
    from agv_perception.storage_control import validate_storage_config
    validate_storage_config(p)


def read_config(path):
    config = read_work_config(path)
    config['storage']['vision'].setdefault('stack_target_uv_px', deepcopy(config['storage']['vision']['target_uv_px']))
    validate(config)
    return config


def save_parameters(path, config, values, area='coarse', production_paths=None):
    if set(values) != {item['path'] for item in area_fields(area)}:
        raise ValueError('参数字段不完整或包含未知字段，请刷新页面')
    candidate = deepcopy(config)
    saved_document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if 'motion_config' in saved_document:
        candidate['motion_config'] = str((Path(path).parent / saved_document['motion_config']).resolve())
    for name, value in values.items():
        parts = name.split('.')
        section = candidate[area]
        for key in parts[:-1]:
            section = section[key]
        section[parts[-1]] = value
    areas = {area}
    for group in (('gantry_motion',) if area == 'raw' else SHARED_GROUPS):
        shared = ('raw', 'coarse', 'storage') if group == 'gantry_motion' else ('coarse', 'storage')
        for other in shared:
            if candidate[other][group] != candidate[area][group]:
                candidate[other][group] = deepcopy(candidate[area][group])
                areas.add(other)
    # The formal executor shares the ring camera and one lease across areas.
    for section, key, shared in (
            ('vision', 'max_frame_age_s', ('coarse', 'storage')),
            ('vision', 'camera_start_timeout_s', ('coarse', 'storage')),
            ('execution', 'lease_timeout_s', ('raw', 'coarse', 'storage'))):
        if area in shared:
            for other in shared:
                if candidate[other][section][key] != candidate[area][section][key]:
                    candidate[other][section][key] = candidate[area][section][key]
                    areas.add(other)
    validate(candidate)
    sync_parameters(path, candidate, sorted(areas), production_paths or {})
    return candidate


def sync_parameters(path, config, areas, production_paths):
    changes = {}
    debug_path = Path(path).resolve()
    original = debug_path.read_text(encoding='utf-8')
    changes[debug_path] = (original, original)
    if 'motion_config' in config:
        motion_path = Path(config['motion_config']).resolve()
        original = motion_path.read_text(encoding='utf-8')
        motion = {group: config['storage'][group] for group in SHARED_GROUPS}
        updated = replace_parameters(original, motion)
        if updated != original:
            changes[motion_path] = (original, updated)
    for area in areas:
        destinations = production_paths.get(area, ())
        if isinstance(destinations, (str, Path)):
            destinations = (destinations,)
        for target in (debug_path, *(Path(p).resolve() for p in destinations)):
            if target not in changes:
                original = target.read_text(encoding='utf-8')
                changes[target] = (original, original)
            original, text = changes[target]
            updates = {area: area_settings(config, area)}
            if 'motion_config' in config:
                updates['motion_config'] = config['motion_config']
            text = replace_parameters(text, updates)
            changes[target] = (original, text)
    write_configs(changes)


def parse_order(value):
    if not isinstance(value, str) or sorted(value) != ['1', '2', '3']:
        raise ValueError('顺序必须是 1、2、3 各一次，例如 213')
    return tuple(int(v) for v in value)


def parse_mode(settings):
    area, visit = settings.get('area', 'coarse'), settings.get('visit', 1)
    if area not in ('raw', 'coarse', 'storage') or type(visit) is not int or visit not in (1, 2):
        raise ValueError('区域须为原料、粗加工或暂存，到达次数须为 1 或 2')
    return area, visit if area == 'storage' else 1


def parse_storage_order(value):
    if (not isinstance(value, str) or len(value) != 3 or len(set(value)) != 3 or
            any(c not in '123456' for c in value)):
        raise ValueError('颜色须为 1～6 中三个不同编号，例如 516')
    return tuple(int(v) for v in value)


def parse_task(settings):
    area, visit = parse_mode(settings)
    if area == 'raw':
        return area, visit, parse_storage_order(settings.get('order')), {}
    if area == 'coarse' or visit == 1:
        return area, visit, parse_order(settings.get('order')), {}
    order = parse_storage_order(settings.get('order'))
    layout = parse_storage_order(settings.get('layout'))
    if set(order) != set(layout):
        raise ValueError('码垛顺序与第一层布局须包含相同的三种颜色')
    return area, visit, order, {color: ring for ring, color in enumerate(layout, 1)}
