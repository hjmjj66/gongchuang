"""Supervise the existing chassis bridge and the standalone work-area debug controller."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parents[1]


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    params = [get('cboard_config')]
    if get('serial_port'):
        params.append({'serial_port': get('serial_port')})
    bridge = Node(package='agv_cboard', executable='cboard_bridge_node', name='cboard_bridge_node',
                  parameters=params, output='screen')
    cmd = ['/usr/bin/python3', '-m', 'coarse_debug.node', '--config', get('config')]
    for name in ('host', 'port'):
        if get(name):
            cmd.extend(['--'+name, get(name)])
    env = {'PYTHONPATH': os.pathsep.join([str(TOOL), str(ROOT/'tools/route_debug'),
                                         str(ROOT/'ros2_ws/src/agv_perception'), os.environ.get('PYTHONPATH', '')]),
           'OPENBLAS_NUM_THREADS': '1'}
    controller = ExecuteProcess(cmd=cmd, additional_env=env, output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p,
                on_exit=[EmitEvent(event=Shutdown(reason='调试子进程退出，统一停止'))]))
                for p in (bridge, controller)]
    return handlers+[bridge, controller]


def generate_launch_description():
    defaults = dict(config=str(TOOL/'config/debug.yaml'),
                    cboard_config=str(ROOT/'ros2_ws/src/agv_cboard/config/cboard.yaml'),
                    serial_port='', host='', port='')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]+
                             [OpaqueFunction(function=setup)])
