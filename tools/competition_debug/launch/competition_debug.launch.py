"""Reuse supervised planner/bridge startup; a separate controller owns this run."""

import importlib.util
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parents[1]


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    mission = yaml.safe_load(Path(get('mission_config')).read_text())['mission_node']['ros__parameters']
    use_lidar = mission.get('use_lidar', True)
    if type(use_lidar) is not bool:
        raise ValueError('use_lidar 必须为 YAML 布尔值')
    extra = []
    if use_lidar:
        path = ROOT/'ros2_ws/src/agv_obstacles/launch/obstacles.launch.py'
        extra.append(PythonLaunchDescriptionSource(str(path)).get_launch_description(context))
    for name in ('raw_config', 'coarse_config', 'storage_config'):
        if not Path(get(name)).is_file():
            raise ValueError(name+' 文件不存在')
    spec = importlib.util.spec_from_file_location('route_debug_launch', ROOT/'tools/route_debug/launch/route_debug.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The route launch forwards --work-config; full-run uses formal per-area files.
    context.launch_configurations['work_config'] = get('coarse_config')
    actions = module.setup(context, controller_module='competition_debug.node', pythonpaths=(TOOL,),
        controller_arguments=('raw_config', 'coarse_config', 'storage_config', 'cboard_config'))
    return extra + actions


def generate_launch_description():
    defaults = dict(host='0.0.0.0', port='8087', serial_port='', screen_port='',
        debug_config=str(ROOT/'tools/route_debug/config/debug.yaml'))
    for name, package in (('planner', 'agv_planning'), ('mission', 'agv_mission'), ('cboard', 'agv_cboard'),
                          ('screen', 'agv_screen'), ('qr', 'agv_perception'), ('raw', 'agv_perception'),
                          ('coarse', 'agv_perception'), ('storage', 'agv_perception')):
        defaults[name+'_config'] = str(ROOT/'ros2_ws/src'/package/'config'/(name+'.yaml'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
