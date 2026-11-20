"""HTTP/ROS/PTY route with station alignment; no physical devices are opened."""

from agv_perception.work_config import read_work_config

import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen
import uuid

import yaml


@unittest.skipUnless(os.environ.get('AGV_ROUTE_DEBUG_ROS_TEST') == '1', 'opt in to isolated ROS/PTY test')
class WorkAlignmentRosTests(unittest.TestCase):
    def test_enabled_route_aligns_three_times_and_stop_aborts_work(self):
        import sys
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG
        from agv_perception.coarse_control import Observation
        from agv_planning.route_planner_node import RoutePlannerNode
        from agv_planning.planner import EDGES
        from route_debug.node import DebugNode, PREFIX
        from route_debug.work_alignment import WorkAlignment
        from test_coarse_ros_pty import ArmBoardPeer

        root = Path(__file__).resolve().parents[3]
        peer = ArmBoardPeer()
        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init()
        nodes = []
        executor = None
        peer.thread.start()
        try:
            with tempfile.TemporaryDirectory(prefix='route_alignment_') as temp:
                folder = Path(temp)
                config = read_work_config((root/'tools/coarse_debug/config/debug.yaml'))
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                for area in ('coarse', 'storage'):
                    p = config[area]
                    p['vision']['target_uv_px'] = [100., 100.]
                    p['angle_alignment']['pid']['kp'] = 2.
                    p['angle_alignment']['max_wz_rad_s'] = .3
                    p['base_servo'].update(tolerance_px=1, max_vx_mps=.2, max_vy_mps=.2)
                    for axis in ('u_pid', 'v_pid'):
                        p['base_servo'][axis]['kp'] = .02
                    p['exit_motion'].update(kp=6, max_vx_mps=.2)
                    p['execution']['feedback_timeout_s'] = .5
                work_file = folder/'work.yaml'
                work_file.write_text(yaml.safe_dump(config))
                debug_config = yaml.safe_load((root/'tools/route_debug/config/debug.yaml').read_text())
                debug_config.update(wait_s=.05, status_timeout_s=.5)
                debug_file = folder/'debug.yaml'
                debug_file.write_text(yaml.safe_dump(debug_config))
                mission = {'mission_node': {'ros__parameters': {'parking_offset_mm': 40}}}
                mission_file = folder/'mission.yaml'
                mission_file.write_text(yaml.safe_dump(mission))
                params = dict(default_edge_length_mm=150, translation_pid_kp=8., translation_pid_kd=0.,
                              angle_pid_kp=8., angle_pid_kd=0., angle_max_speed_deg_s=120.,
                              turn_arrival_tolerance_deg=1., debug_allow_any_pose=True,
                              debug_publish_progress=True)
                params.update({f'edge_length_mm.{min(a,b)}_{max(a,b)}':150 for a,b in EDGES})
                params.update({f'work_heading.node_{n}':h for n,h in ((2,'RIGHT'),(4,'LEFT'),(6,'RIGHT'),(8,'DOWN'))})
                params.update({f'{key}':PREFIX+'/'+value for key,value in (
                    ('plan_service','plan'), ('route_topic','path'), ('control_topic','control'),
                    ('execution_status_topic','status'), ('blocked_edges_topic','map'), ('debug_progress_topic','progress'))})
                planner_file = folder/'planner.yaml'
                planner_file.write_text(yaml.safe_dump({'route_planner_node': {'ros__parameters':params}}))
                planner = RoutePlannerNode(parameter_overrides=[Parameter(k, value=v) for k,v in params.items()],
                    cli_args=['--ros-args', *sum((['-r', f'/route/{key}:={PREFIX}/{key}']
                    for key in ('get_runtime','stop','reset_context')), []),
                    '-r', '/mission/heartbeat:='+PREFIX+'/heartbeat'])
                nodes.append(planner)
                bridge = CBoardBridgeNode(parameter_overrides=[Parameter('serial_port', value=peer.path)])
                nodes.append(bridge)
                debug = DebugNode(SimpleNamespace(qr_config=str(root/'ros2_ws/src/agv_perception/config/qr.yaml'), debug_config=str(debug_file), planner_config=str(planner_file),
                    mission_config=str(mission_file), screen_config=str(root/'ros2_ws/src/agv_screen/config/screen.yaml'),
                    screen_port=None, work_config=str(work_file), host='127.0.0.1', port=0))
                nodes.append(debug)

                class Vision:
                    ready, error = True, ''
                    task_id = ''
                    sequence = 0

                    def latest(self):
                        task = debug.work.task
                        if task.task_id != self.task_id:
                            self.task_id = task.task_id
                            self.origin = peer.x, peer.y, peer.yaw-.04
                        x,y,yaw = self.origin
                        dx,dy = peer.x-x, peer.y-y
                        u = 100.+(dx*math.cos(yaw)+dy*math.sin(yaw)-.008)*200.
                        v = 100.-(-dx*math.sin(yaw)+dy*math.cos(yaw)+.008)*200.
                        rings = {n:(u+d,v+d*math.tan(peer.yaw-yaw)) for n,d in ((1,-80),(2,0),(3,80))}
                        self.sequence += 1
                        return Observation('ring', self.sequence, time.monotonic(), (201,201), rings)

                    def close(self):
                        pass

                debug.work = WorkAlignment(debug, work_file, vision=Vision())
                executor = CommandExecutor(bridge._command_sub)
                for node in nodes:
                    executor.add_node(node)
                base = f'http://127.0.0.1:{debug.web.http.server_port}'
                owner = 'alignment-browser'
                last_heartbeat = 0.
                tasks = {}

                def request(route, payload=None):
                    req = Request(base+route, data=None if payload is None else json.dumps(payload).encode(),
                                  headers={'Content-Type':'application/json'})
                    with urlopen(req, timeout=2) as response:
                        return json.load(response)

                def pump(predicate, timeout=60):
                    nonlocal last_heartbeat
                    until = time.monotonic()+timeout
                    while time.monotonic() < until:
                        if time.monotonic()-last_heartbeat > .2:
                            request('/api/heartbeat', dict(client=owner))
                            last_heartbeat = time.monotonic()
                        executor.spin_once(timeout_sec=.003)
                        if debug.work.task:
                            tasks[debug.work.task.task_id] = debug.work.task
                        if predicate():
                            return
                        self.assertNotEqual(debug.session.state, 'FAULT', debug.session.reason)
                    self.fail(f'timeout {debug.session.state}: {debug.session.reason}; '
                              f'diagnostic={debug.work.task and debug.work.task.diagnostic}; peer={peer.error}')

                def command(action, settings=None):
                    operation = uuid.uuid4().hex
                    request('/api/command', dict(client=owner, operation_id=operation,
                            action=action, settings=settings or {}))
                    pump(lambda:not debug.web.operations[(owner,operation)][1]['pending'])
                    result = debug.web.operations[(owner,operation)][1]
                    self.assertTrue(result['ok'], result)

                pump(lambda:debug.session.health() == '' and debug.services_ready() and debug.work.arm is not None)
                settings = dict(mode='competition', start=1, goal=3, work_alignment=True)
                command('start', settings)
                pump(lambda:debug.session.state == 'FINISHED')
                self.assertEqual([p[0] for p in peer.arm_actions], [3,3,2,3,2,3,3,2])
                self.assertEqual(len(tasks), 3)
                self.assertEqual([t.final_alignment for t in tasks.values()], ['ALIGNED', 'NOT_RUN', 'ALIGNED'])
                for task in tasks.values():
                    self.assertEqual(task.entry_alignment, 'ALIGNED')
                    self.assertEqual(task.state, 'DONE')
                    self.assertTrue(task.returned)
                    self.assertLessEqual(math.hypot(*task.diagnostic['error_xy']), .001)
                self.assertFalse(any(mid == ARM_JOG for _,mid,_ in peer.frames))
                pump(lambda:request('/api/state')['state'] == 'FINISHED')
                snapshot = request('/api/state')
                self.assertEqual(snapshot['state'], 'FINISHED')
                self.assertNotIn('camera_car_offset', snapshot['config'])
                command('start', settings)
                pump(lambda:debug.session.state == 'WORKING' and debug.work.task.state == 'WORK_ENTRY')
                command('stop')
                pump(lambda:debug.session.state == 'STOPPED')
                commands = [payload[0] for _,mid,payload in peer.frames if mid == ARM_CMD]
                self.assertEqual(commands[-1], 0)
                self.assertTrue(debug.work.released())
                self.assertIsNone(peer.error)
        finally:
            if nodes and isinstance(nodes[-1], DebugNode):
                nodes[-1].close()
            if executor:
                executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            peer.close()
            rclpy.shutdown()
            sys.setswitchinterval(switch_interval)
