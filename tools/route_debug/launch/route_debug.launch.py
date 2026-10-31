"""Supervised real planner + real CDC bridge + standalone debug controller."""

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
PREFIX = '/tools/route_debug'


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    planner = Node(package='agv_planning', executable='route_planner_node',
        name='route_planner_node', output='screen',
        parameters=[get('planner_config'), {'debug_allow_any_pose': True,
                    'debug_publish_progress': True,
                    'work_heading.node_2': 'DOWN', 'work_heading.node_4': 'LEFT',
                    'work_heading.node_6': 'RIGHT', 'work_heading.node_8': 'DOWN',
                    'plan_service': PREFIX + '/plan', 'route_topic': PREFIX + '/path',
                    'control_topic': PREFIX + '/control', 'execution_status_topic': PREFIX + '/status',
                    'blocked_edges_topic': PREFIX + '/map', 'odometry_topic': '/C/odometry',
                    'debug_progress_topic': PREFIX + '/progress'}],
        remappings=[('/route/get_runtime', PREFIX + '/get_runtime'),
                    ('/route/stop', PREFIX + '/stop'), ('/route/reset_context', PREFIX + '/reset_context'),
                    ('/mission/heartbeat', PREFIX + '/heartbeat')])
    board_params = [get('cboard_config'), {'control_topic': '/C/controll',
                    'odometry_topic': '/C/odometry', 'status_topic': '/C/status'}]
    if get('serial_port'):
        board_params.append({'serial_port': get('serial_port')})
    board = Node(package='agv_cboard', executable='cboard_bridge_node',
                 name='cboard_bridge_node', parameters=board_params, output='screen')
    env = {'PYTHONPATH': str(TOOL) + os.pathsep + os.environ.get('PYTHONPATH', '')}
    controller_cmd = ['/usr/bin/python3', '-m', 'route_debug.node',
                      '--planner-config', get('planner_config'), '--debug-config', get('debug_config'),
                      '--mission-config', get('mission_config')]
    controller_cmd.extend(['--screen-config', get('screen_config')])
    if get('screen_port'):
        controller_cmd.extend(['--screen-port', get('screen_port')])
    for name in ('host', 'port'):
        if get(name):
            controller_cmd.extend(['--' + name, get(name)])
    controller = ExecuteProcess(cmd=controller_cmd, additional_env=env, output='screen')
    actions = []
    for process in (planner, board, controller):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='调试子进程退出，统一停止'))])))
    return actions + [planner, board, controller]


def generate_launch_description():
    defaults = dict(planner_config=str(ROOT / 'ros2_ws/src/agv_planning/config/planner.yaml'),
                    mission_config=str(ROOT / 'ros2_ws/src/agv_mission/config/mission.yaml'),
                    cboard_config=str(ROOT / 'ros2_ws/src/agv_cboard/config/cboard.yaml'),
                    debug_config=str(TOOL / 'config/debug.yaml'),
                    screen_config=str(ROOT / 'ros2_ws/src/agv_screen/config/screen.yaml'),
                    screen_port='',
                    serial_port='', host='', port='')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
