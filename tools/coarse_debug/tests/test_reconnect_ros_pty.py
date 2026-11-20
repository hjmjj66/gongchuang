"""Recover manual-start readiness after real PTY removal/replacement; no hardware."""

import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen
import uuid

import yaml

from test_pose_calibration_ros import PoseVision


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST') == '1', 'opt in to isolated HTTP/ROS/PTY test')
class ReconnectRosTests(unittest.TestCase):
    def test_reconnect_resumes_same_tracking_task_with_preserved_board_coordinates(self):
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from coarse_debug.config import read_config
        from coarse_debug.node import DebugNode
        from test_coarse_ros_pty import ArmBoardPeer

        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID', '188')))
        peer = bridge = debug = executor = None
        try:
            with tempfile.TemporaryDirectory(prefix='reconnect_web_') as folder:
                folder = Path(folder)
                port = folder/'cboard'
                peer = ArmBoardPeer()
                peer.yaw = 0.
                port.symlink_to(peer.path)
                peer.thread.start()
                config = read_config(Path(__file__).resolve().parents[1]/'config/debug.yaml')
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                config['coarse']['vision']['target_uv_px'] = [100., 100.]
                config['coarse']['execution']['feedback_timeout_s'] = .5
                path = folder/'debug.yaml'
                path.write_text(yaml.safe_dump(config, allow_unicode=True))
                bridge = CBoardBridgeNode(parameter_overrides=[
                    Parameter('serial_port', value=str(port)), Parameter('reconnect_interval_s', value=.05)])
                debug = DebugNode(SimpleNamespace(config=str(path), host='127.0.0.1', port=0), vision=PoseVision(peer))
                executor = CommandExecutor(bridge._command_sub)
                executor.add_node(bridge)
                executor.add_node(debug)
                base = f'http://127.0.0.1:{debug.web.http.server_port}'
                client = 'reconnect-browser'

                def request(route, payload=None):
                    req = Request(base+route, data=None if payload is None else json.dumps(payload).encode(),
                                  headers={'Content-Type': 'application/json'})
                    with urlopen(req, timeout=2.) as response:
                        return json.loads(response.read())

                def pump(predicate, timeout=8.):
                    until = time.monotonic()+timeout
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=.005)
                        if predicate():
                            return
                    self.fail(f'timeout: {debug.web.snapshot()}; arm={bridge._arm.state}')

                def command(action, settings=None):
                    operation = uuid.uuid4().hex
                    request('/api/command', dict(client=client, operation_id=operation,
                                                 action=action, settings=settings or {}))
                    pump(lambda: not debug.web.operations[(client, operation)][1]['pending'])
                    self.assertTrue(debug.web.operations[(client, operation)][1]['ok'],
                                    debug.web.operations[(client, operation)][1])

                pump(lambda: debug._can_start(debug.vision.latest()))
                command('select', dict(mode='calibration', calibration_type='shared_pose'))
                command('preset_adjust')
                pump(lambda: debug.task.state == 'ALIGN_ENTRY' and debug._arm_idle())
                pump(lambda: debug.reference_ready is not None)
                command('preset_track', dict(target=3))
                pump(lambda: debug.task.servo_enabled and peer.jog)
                for attempt in range(2):
                    interrupted = debug.task
                    old_session = debug.binding
                    old_peer = peer
                    peer.close()
                    peer = None
                    port.unlink()
                    pump(lambda: not debug.board.connected and getattr(debug, '_link_paused_at', None) is not None)
                    self.assertFalse(interrupted.finished)
                    self.assertTrue(debug.active)
                    self.assertFalse(debug._can_start(debug.vision.latest()))

                    peer = ArmBoardPeer()
                    for name in ('clock_start', 'x', 'y', 'yaw', 'arm_fb', 'arm_yaw'):
                        setattr(peer, name, getattr(old_peer, name))
                    debug.vision.peer = peer
                    port.symlink_to(peer.path)
                    peer.thread.start()
                    pump(lambda: getattr(debug, '_link_paused_at', None) is None and peer.jog)
                    self.assertEqual(debug.binding, old_session)
                    self.assertIs(debug.task, interrupted)
                    self.assertTrue(debug.active)
                    self.assertFalse(debug.task.finished)
                    self.assertFalse(any(mid == ARM_CMD for _, mid, _ in peer.frames))
                    self.assertIsNone(peer.error)
                command('stop')
                pump(lambda: debug._arm_idle())
                command('select', dict(mode='task', area='coarse', visit=1))
                pump(lambda: request('/api/state')['can_start'])
                command('start', dict(area='coarse', visit=1, order='213'))
                pump(lambda: any(mid == ARM_CMD and payload[0] == 3 for _, mid, payload in peer.frames))
                self.assertTrue(debug.active)
                self.assertIsNone(peer.error)
        finally:
            if debug:
                debug.close()
            if executor:
                executor.shutdown()
            if debug:
                debug.destroy_node()
            if bridge:
                bridge.destroy_node()
            if peer:
                peer.close()
            rclpy.shutdown()
            sys.setswitchinterval(switch_interval)
