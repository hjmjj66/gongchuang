"""One owner for each device; any child exit shuts down the complete launch."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .preflight import preflight


CONFIGS = {
    'mission': ('agv_mission', 'mission.yaml'),
    'screen': ('agv_screen', 'screen.yaml'),
    'planner': ('agv_planning', 'planner.yaml'),
    'cboard': ('agv_cboard', 'cboard.yaml'),
    'qr': ('agv_perception', 'qr.yaml'),
    'raw': ('agv_perception', 'raw.yaml'),
    'coarse': ('agv_perception', 'coarse.yaml'),
    'storage': ('agv_perception', 'storage.yaml'),
}


def setup(context):
    paths = {name: LaunchConfiguration(name + '_config').perform(context) for name in CONFIGS}
    for name, path in paths.items():
        if not Path(path).is_file():
            raise RuntimeError(f'{name}_config 文件不存在：{path}')
    with open(paths['mission'], encoding='utf-8') as stream:
        mission = yaml.safe_load(stream)['mission_node']['ros__parameters']
    use_lidar = mission.get('use_lidar', True)
    if type(use_lidar) is not bool:
        raise ValueError('mission.yaml 中 use_lidar 必须为 YAML 布尔值 true/false')

    extra = []
    if use_lidar:
        # Load before creating hardware processes. The placeholder fails here;
        # implementing this launch file later requires no change to bringup.
        path = Path(get_package_share_directory('agv_obstacles')) / 'launch/obstacles.launch.py'
        extra.append(PythonLaunchDescriptionSource(str(path)).get_launch_description(context))

    preflight()
    nodes = [
        Node(package='agv_mission', executable='mission_node', name='mission_node', output='screen',
             parameters=[paths['mission'], {'mode': 'competition', 'screen_controlled': True}]),
        Node(package='agv_cboard', executable='cboard_bridge_node', name='cboard_bridge_node', output='screen',
             parameters=[paths['cboard']]),
        Node(package='agv_planning', executable='route_planner_node', name='route_planner_node', output='screen',
             parameters=[paths['planner'], {'result_source': 'REAL', 'debug_allow_any_pose': False}]),
        Node(package='agv_perception', executable='coarse_node', name='coarse_node', output='screen',
             parameters=[{'config': paths['coarse'], 'raw_config': paths['raw'],
                          'storage_config': paths['storage']}]),
        Node(package='agv_screen', executable='screen_node', name='screen_node', output='screen',
             parameters=[paths['screen']]),
        Node(package='agv_perception', executable='qr_node', name='qr_node', output='screen',
             parameters=[paths['qr'], {'wait_for_mission': True, 'show_preview': False,
                                      'result_topic': mission.get('qr_input_topic', '/qr/result')}]),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='比赛节点退出；整套停止，不自动重启或续跑'))]))
        for node in nodes]
    return extra + handlers + nodes


def generate_launch_description():
    arguments = [DeclareLaunchArgument(
        name + '_config',
        default_value=str(Path(get_package_share_directory(package)) / 'config' / filename))
        for name, (package, filename) in CONFIGS.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
