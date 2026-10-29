"""Opt-in full launch/HTTP/ROS/CDC test. Only opens a newly created PTY."""

import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from urllib.request import Request, urlopen
import uuid

import yaml


ROOT = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parents[1]


class MovingBoard:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.closed = threading.Event()
        self.frames = []
        self.send_odom = True
        self.source = 2
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.error = None

    def run(self):
        from agv_cboard.protocol import (CHASSIS_CMD, CHASSIS_STRUCT, ODOM, ODOM_STRUCT,
                                        STATUS, STATUS_STRUCT, FrameParser, encode_frame)
        parser = FrameParser()
        x = y = yaw = 0.
        command = (0., 0., 0., 0)
        previous = time.monotonic()
        last_command = previous
        next_status = previous
        board_start = previous
        try:
            while not self.closed.is_set():
                now = time.monotonic()
                dt, previous = min(.03, now - previous), now
                try:
                    data = os.read(self.master, 4096)
                except BlockingIOError:
                    data = b''
                for mid, payload in parser.feed(data):
                    if mid == CHASSIS_CMD:
                        command = CHASSIS_STRUCT.unpack(payload)
                        last_command = now
                        self.frames.append((now, command))
                vx = vy = wz = 0.
                if command[3] == 1 and self.source == 2 and now - last_command < .2:
                    vx, vy, wz, _ = command
                    x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
                    y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
                    yaw += wz * dt
                tick = int((now - board_start) * 1000)
                if now >= next_status:
                    os.write(self.master, encode_frame(STATUS, STATUS_STRUCT.pack(
                        tick, 1, 0, 15, 0, 0., 0., 0., self.source)))
                    next_status = now + .04
                if self.send_odom:
                    os.write(self.master, encode_frame(ODOM, ODOM_STRUCT.pack(
                        tick, x, y, yaw, vx, vy, wz, 0x0D)))
                self.closed.wait(.01)
        except Exception as exc:
            self.error = exc

    def start(self):
        self.thread.start()

    def close(self):
        self.closed.set()
        self.thread.join(2)
        os.close(self.master)
        os.close(self.slave)


@unittest.skipUnless(os.environ.get('AGV_ROUTE_DEBUG_ROS_TEST') == '1',
                     'set AGV_ROUTE_DEBUG_ROS_TEST=1 in an isolated ROS domain')
class RosPtyTests(unittest.TestCase):
    def test_browser_to_real_planner_and_bridge(self):
        board = MovingBoard()
        process = None
        succeeded = False
        with tempfile.TemporaryDirectory(prefix='route_debug_pty_') as temp:
            directory = Path(temp)
            planner = yaml.safe_load((ROOT / 'ros2_ws/src/agv_planning/config/planner.yaml').read_text())
            params = planner['route_planner_node']['ros__parameters']
            params.update(default_edge_length_mm=150, translation_fast_mps=.5,
                          translation_slow_mps=.15, translation_deceleration_mm=40.,
                          turn_fast_deg_s=360., turn_slow_deg_s=180.,
                          turn_arrival_tolerance_deg=1., odometry_timeout_s=.5)
            planner_file = directory / 'planner.yaml'
            planner_file.write_text(yaml.safe_dump(planner))
            mission = yaml.safe_load((ROOT / 'ros2_ws/src/agv_mission/config/mission.yaml').read_text())
            mission['mission_node']['ros__parameters']['parking_offset_mm'] = 275
            mission_file = directory / 'mission.yaml'
            mission_file.write_text(yaml.safe_dump(mission))
            with socket.socket() as port_socket:
                port_socket.bind(('127.0.0.1', 0))
                port = port_socket.getsockname()[1]
            base = f'http://127.0.0.1:{port}'
            client = uuid.uuid4().hex
            alive = threading.Event()
            alive.set()

            def request(path, payload=None):
                data = json.dumps(dict(client=client, **payload)).encode() if payload is not None else None
                req = Request(base + path, data=data, headers={'Content-Type': 'application/json'})
                with urlopen(req, timeout=2) as response:
                    return json.load(response)

            def snapshot():
                return request('/api/state')

            def wait_for(predicate, timeout=10):
                deadline = time.monotonic() + timeout
                last = None
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        self.fail('launch exited early')
                    try:
                        last = snapshot()
                        if predicate(last):
                            return last
                    except (OSError, ValueError):
                        pass
                    time.sleep(.05)
                self.fail('timeout, last state=' + json.dumps(last, ensure_ascii=False))

            def command(action, settings):
                operation_id = uuid.uuid4().hex
                payload = dict(action=action, settings=settings, operation_id=operation_id)
                request('/api/command', payload)
                # Exact retry must reuse the operation, not initialize twice.
                request('/api/command', payload)
                result = wait_for(lambda s: any(op.get('operation_id') == operation_id and not op.get('pending') for op in s.get('operations', [])))
                operation = next(op for op in result['operations'] if op['operation_id'] == operation_id)
                self.assertTrue(operation['ok'], operation)

            def heartbeats():
                while alive.is_set():
                    try:
                        request('/api/heartbeat', {})
                    except OSError:
                        pass
                    time.sleep(.25)

            log = (directory / 'launch.log').open('w+')
            heartbeat_thread = threading.Thread(target=heartbeats, daemon=True)
            try:
                board.start()
                env = dict(os.environ, ROS_LOG_DIR=str(directory / 'ros_logs'))
                process = subprocess.Popen([str(TOOL / 'start.sh'), 'host:=127.0.0.1', f'port:={port}',
                    f'serial_port:={board.path}', f'planner_config:={planner_file}',
                    f'mission_config:={mission_file}'],
                    env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                initial = wait_for(lambda s: s.get('preflight_reason') == '', 15)
                self.assertEqual(initial['config']['translation_fast_mps'], .5)
                self.assertEqual(initial['config']['parking_offset_mm'], 275)
                self.assertEqual(initial['config']['mission_source'], str(mission_file))
                heartbeat_thread.start()
                with urlopen(base + '/', timeout=2) as response:
                    self.assertIn('路线与底盘调试', response.read().decode())
                with urlopen(base + '/map.png', timeout=2) as response:
                    self.assertEqual(response.read(8), b'\x89PNG\r\n\x1a\n')
                with urlopen(base + '/api/events', timeout=2) as response:
                    self.assertTrue(response.readline().startswith(b'data: '))
                # Real-world regression: node 1 -> 2, nose DOWN, is pure +vx.
                frame_start = len(board.frames)
                command('start', dict(mode='free', start=1, goal=2, heading='DOWN', align=False))
                forward = wait_for(lambda s: s['state'] in ('FINISHED', 'FAULT'), 10)
                self.assertEqual(forward['state'], 'FINISHED', forward)
                moving = [cmd for _, cmd in board.frames[frame_start:] if cmd[3] == 1]
                self.assertTrue(moving)
                for vx, vy, wz, mode in moving:
                    self.assertGreater(vx, 0.)
                    self.assertEqual((vy, wz), (0., 0.))
                # Optional free-mode exits honor the selected heading, then align only at node 2.
                for start, heading, sx, sy in ((1, 'RIGHT', -1, -1), (3, 'LEFT', 1, -1)):
                    frame_start = len(board.frames)
                    command('start', dict(mode='free', start=start, goal=2, heading=heading,
                                          align=True, exit_start=True))
                    exiting = wait_for(lambda s: s['state'] == 'NAVIGATING' and
                                       s['parking_maneuver'] == 'EXIT' and s['plan'])
                    self.assertEqual(exiting['route_count'], 2)
                    self.assertEqual(exiting['plan']['nodes'], [start])
                    self.assertEqual(exiting['plan']['start_heading'], heading)
                    self.assertEqual(exiting['plan']['end_heading'], heading)
                    self.assertEqual(exiting['plan']['actions'][0]['distance_mm'], 275)
                    finished = wait_for(lambda s: s['state'] in ('FINISHED', 'FAULT'), 12)
                    self.assertEqual(finished['state'], 'FINISHED', finished)
                    self.assertEqual(finished['route_index'], 1)
                    self.assertEqual(finished['parking_maneuver'], '')
                    self.assertEqual(finished['plan']['nodes'], [start, 2])
                    self.assertEqual(finished['plan']['end_heading'], 'DOWN')
                    self.assertEqual(finished['command'], [0., 0., 0.])
                    diagonal = [cmd for _, cmd in board.frames[frame_start:]
                                if cmd[3] == 1 and cmd[0] != 0. and cmd[1] != 0.]
                    self.assertTrue(diagonal)
                    self.assertTrue(all(vx * sx > 0. and vy * sy > 0. and wz == 0.
                                        for vx, vy, wz, _ in diagonal))
                free = dict(mode='free', start=7, goal=9, heading='LEFT', align=False)
                command('start', free)
                finished = wait_for(lambda s: s['state'] in ('FINISHED', 'FAULT'), 10)
                self.assertEqual(finished['state'], 'FINISHED', finished)
                self.assertGreater(len(finished['trace']), 1)
                self.assertEqual(finished['plan']['nodes'], [7,8,9])
                self.assertTrue(any(frame[1][3] == 1 for frame in board.frames))
                # Full run preserves the real three-second waits.
                race = dict(mode='competition', start=3, goal=1, heading='UP', align=False)
                frame_start = len(board.frames)
                command('start', race)
                exiting = wait_for(lambda s: s['state'] == 'NAVIGATING' and
                                   s['parking_maneuver'] == 'EXIT' and s['plan'])
                self.assertEqual(exiting['route_index'], 0)
                self.assertEqual(exiting['route_count'], 10)
                self.assertEqual(exiting['plan']['nodes'], [3])
                self.assertEqual(exiting['plan']['actions'][0]['distance_mm'], 275)
                self.assertEqual(exiting['plan']['actions'][0]['opcode'], 7)
                self.assertEqual(len(exiting['plan']['display_points']), 2)
                finished = wait_for(lambda s: s['state'] in ('FINISHED', 'FAULT'), 55)
                self.assertEqual(finished['state'], 'FINISHED', finished)
                self.assertEqual(finished['route_index'], 9)
                waits = [event for event in finished['events'] if '等待' in event['message']]
                self.assertEqual(len(waits), 7)
                self.assertGreaterEqual(finished['elapsed_s'], 21)
                self.assertEqual(finished['plan']['goal'], 1)
                self.assertEqual(finished['plan']['nodes'], [1])
                self.assertEqual(finished['plan']['actions'][0]['opcode'], 9)
                self.assertEqual(finished['plan']['actions'][0]['distance_mm'], 275)
                self.assertEqual(finished['parking_maneuver'], 'ENTER')
                diagonal = [cmd for _, cmd in board.frames[frame_start:]
                            if cmd[3] == 1 and cmd[0] != 0. and cmd[1] != 0.]
                self.assertTrue(any(vx > 0. and vy > 0. and wz == 0. for vx, vy, wz, _ in diagonal))
                self.assertTrue(any(vx < 0. and vy > 0. and wz == 0. for vx, vy, wz, _ in diagonal))
                self.assertEqual(finished['command'], [0.,0.,0.])
                # New run, stop while waiting, reset, and execute zero-distance alignment.
                command('start', race)
                wait_for(lambda s: s['state'] == 'WAITING', 8)
                command('stop', race)
                wait_for(lambda s: s['state'] == 'STOPPED')
                command('reset', free)
                wait_for(lambda s: s['state'] == 'READY')
                same = dict(mode='free', start=2, goal=2, heading='UP', align=True)
                command('start', same)
                finished = wait_for(lambda s: s['state'] in ('FINISHED', 'FAULT'))
                self.assertEqual(finished['state'], 'FINISHED', finished)
                self.assertEqual(finished['plan']['end_heading'], 'DOWN')
                # Feedback interruption terminates the task; bridge health recovers automatically.
                command('start', race)
                wait_for(lambda s: s['state'] == 'WAITING', 8)
                board.send_odom = False
                interrupted = wait_for(lambda s: s['state'] == 'FAULT', 4)
                self.assertEqual(interrupted['command'], [0.,0.,0.])
                board.send_odom = True
                wait_for(lambda s: s.get('board', {}).get('odometry_valid') and s['board_fresh'], 4)
                command('reset', free)
                recovered = wait_for(lambda s: s['state'] in ('READY', 'FAULT'))
                self.assertEqual(recovered['state'], 'READY', recovered)
                # RC takeover and PC re-entry cancel the old task, reset origin,
                # and allow a NEW start with selected pose (no clear-fault RPC).
                command('start',race)
                wait_for(lambda s:s['state']=='WAITING',8)
                old_session=snapshot()['odom_session_id']
                board.source=1
                wait_for(lambda s:s['state']=='FAULT',4)
                board.source=2
                reset=wait_for(lambda s:s['odom_session_id']!=old_session and not s['preflight_reason'],4)
                self.assertEqual(reset['state'],'FAULT')
                self.assertEqual(reset['command'],[0.,0.,0.])
                command('start',free)
                restarted=wait_for(lambda s:s['state'] in ('FINISHED','FAULT'),10)
                self.assertEqual(restarted['state'],'FINISHED',restarted)
                # Browser heartbeat loss cancels a running route (no resume).
                command('start', race)
                wait_for(lambda s: s['state'] == 'WAITING', 8)
                alive.clear()
                lost = wait_for(lambda s: s['state'] == 'FAULT', 12)
                self.assertIn('页面失联', lost['reason'])
                self.assertEqual(lost['command'], [0.,0.,0.])
                self.assertIsNone(board.error)
                succeeded = True
            finally:
                alive.clear()
                if heartbeat_thread.is_alive():
                    heartbeat_thread.join(3)
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(3)
                board.close()
                log.seek(0)
                output = log.read()
                log.close()
                if not succeeded or (process is not None and process.returncode not in (0, -signal.SIGINT)):
                    print(output[-18000:])

    def test_formal_planner_keeps_original_reset_rules(self):
        import rclpy
        from agv_planning.route_planner_node import RoutePlannerNode
        from agv_interfaces.srv import ResetExecutor
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init()
        planner = client_node = executor = None
        try:
            planner = RoutePlannerNode()
            client_node = rclpy.create_node('formal_reset_contract_test')
            executor = SingleThreadedExecutor()
            executor.add_node(planner)
            executor.add_node(client_node)
            client = client_node.create_client(ResetExecutor, '/route/reset_context')
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            request = ResetExecutor.Request(expected_run_id='', run_id='test-round',
                reset_id=uuid.uuid4().hex, home_node=7, initial_heading='LEFT')
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=3)
            result = future.result()
            self.assertFalse(result.accepted)
            self.assertEqual(result.runtime.run_id, '')
            self.assertIsNone(planner._progress_pub)
            request.home_node, request.initial_heading = 1, 'DOWN'
            request.reset_id = uuid.uuid4().hex
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=3)
            self.assertTrue(future.result().accepted)
            self.assertEqual(future.result().runtime.run_id, 'test-round')
        finally:
            if executor:
                executor.shutdown()
            for node in (client_node, planner):
                if node:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
