"""Opt-in HTTP/ROS/serial integration, with synthetic vision and a PTY board only."""

import json
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen
import uuid

import yaml

from agv_perception.work_control import Observation


class TrackingVision:
    ready, error, kind, hidden = True, '', 'ring', False

    def __init__(self, board):
        self.board, self.sequence = board, 0
        self.geometry_target = 0

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        self.kind = kind
        self.geometry_target = geometry_target

    def latest(self):
        self.sequence += 1
        point = (110.+self.board.arm_yaw*100., 110.+self.board.arm_fb*5.)
        rings = {2:point} if not self.hidden and self.kind == 'ring' else {}
        materials = ((2,point),) if not self.hidden and self.kind == 'material' else ()
        return Observation(self.kind,self.sequence,time.monotonic(),(201,201),rings,materials,
                           geometry_target=self.geometry_target)

    def preview(self):
        return b'tracking-fixture'

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST') == '1', 'opt in to HTTP/ROS/PTY test')
class TrackingRosTests(unittest.TestCase):
    def test_tracking_real_program_handoff_owner_stop_and_disconnect(self):
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from coarse_debug.config import read_config
        from coarse_debug.node import DebugNode
        from test_coarse_ros_pty import ArmBoardPeer

        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','184')))
        peer = ArmBoardPeer()
        bridge = debug = executor = None
        peer.thread.start()
        try:
            with tempfile.TemporaryDirectory(prefix='tracking_web_') as folder:
                config = read_config(Path(__file__).parents[1]/'config/debug.yaml')
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                config['debug']['browser_timeout_s'] = .7
                for area in ('raw','coarse','storage'):
                    config[area]['execution']['feedback_timeout_s'] = .5
                    config[area]['vision']['target_uv_px'] = [100.,100.]
                path = Path(folder)/'debug.yaml'
                path.write_text(yaml.safe_dump(config,allow_unicode=True))
                bridge = CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=peer.path)])
                debug = DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=TrackingVision(peer))
                executor = CommandExecutor(bridge._command_sub)
                executor.add_node(bridge)
                executor.add_node(debug)
                base = f'http://127.0.0.1:{debug.web.http.server_port}'
                client, last_heartbeat = 'tracking-test-page', 0.

                def request(route,payload=None):
                    req = Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                  headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=2.) as response:
                        return json.loads(response.read())

                def pump(predicate,timeout=6.,heartbeat=True):
                    nonlocal last_heartbeat
                    until = time.monotonic()+timeout
                    while time.monotonic()<until:
                        if heartbeat and time.monotonic()-last_heartbeat>.15:
                            request('/api/heartbeat',dict(client=client))
                            last_heartbeat = time.monotonic()
                        executor.spin_once(timeout_sec=.005)
                        if predicate():
                            return
                    self.fail(f'timeout: {debug.task and debug.task.state}; {debug.task and debug.task.reason}; peer={peer.error}')

                def command(action,settings=None,who=client,operation_id=None):
                    operation_id = operation_id or uuid.uuid4().hex
                    payload = dict(client=who,operation_id=operation_id,action=action,settings=settings or {})
                    request('/api/command',payload)
                    pump(lambda:not debug.web.operations[(who,operation_id)][1]['pending'])
                    return debug.web.operations[(who,operation_id)][1]

                pump(lambda:debug._can_start(debug.vision.latest()))
                for area,visit,expected in (('raw',1,4),('coarse',1,6),('storage',1,6),('storage',2,7)):
                    self.assertTrue(command('select',dict(area=area,visit=visit,mode='tracking'))['ok'])
                    pump(lambda:debug._can_start(debug.vision.latest()))
                    offset = len(peer.frames)
                    self.assertTrue(command('start',dict(area=area,visit=visit,mode='tracking',target=2))['ok'])
                    pump(lambda:any(abs(v)>0 for v in debug.task.arm_velocity))
                    self.assertEqual(debug.vision.geometry_target, 2)
                    self.assertIsNone(debug.task.arm_request)
                    self.assertEqual(debug.task.velocity,(0.,0.,0.))
                    self.assertFalse(command('select',dict(area=area,visit=visit,mode='task'))['ok'])
                    self.assertFalse(command('tracking_action',who='different-page')['ok'])
                    debug.vision.hidden = True
                    pump(lambda:debug.task.state == 'TRACK_WAIT' and not peer.jog)
                    self.assertEqual(debug.task.arm_velocity,(0.,0.))
                    debug.vision.hidden = False
                    pump(lambda:debug.task.servo_enabled and peer.jog)
                    operation_id = uuid.uuid4().hex
                    self.assertTrue(command('tracking_action',operation_id=operation_id)['ok'])
                    self.assertTrue(command('tracking_action',operation_id=operation_id)['ok'])
                    pump(lambda:debug.task.finished)
                    self.assertEqual(debug.task.state,'DONE',debug.task.reason)
                    self.assertEqual([data[0] for _,mid,data in peer.frames[offset:] if mid == ARM_CMD],[expected])
                    self.assertFalse(peer.jog)
                    self.assertEqual(debug.task.arm_velocity,(0.,0.))
                    pump(lambda:request('/api/state')['state'] == 'DONE')
                    snapshot = request('/api/state')
                    self.assertEqual(snapshot['mode'],'tracking')
                    self.assertEqual(snapshot['tracking_target'],2)
                    self.assertEqual(len(snapshot['confirmed']),1)

                for failure in ('stop','heartbeat'):
                    pump(lambda:debug._can_start(debug.vision.latest()))
                    self.assertTrue(command('start',dict(area='storage',visit=2,mode='tracking',target=2))['ok'])
                    pump(lambda:debug.task.servo_enabled and peer.jog)
                    if failure == 'stop':
                        self.assertTrue(command('stop')['ok'])
                        self.assertEqual(debug.task.state,'STOPPED')
                    else:
                        pump(lambda:debug.task.finished,heartbeat=False)
                        self.assertIn('页面失联',debug.task.reason)
                    pump(lambda:debug._arm_idle() and not peer.jog)
                    self.assertEqual(debug.task.arm_velocity,(0.,0.))
                for category in ('ring', 'shared_pose'):
                    self.assertTrue(command('select',dict(mode='calibration',calibration_type=category))['ok'])
                    pump(lambda:debug._can_start(debug.vision.latest()))
                    offset = len(peer.frames)
                    self.assertTrue(command('calibration_track_fb')['ok'])
                    pump(lambda:debug.task.servo_enabled and peer.jog)
                    self.assertEqual(debug.vision.geometry_target, 2)
                    self.assertEqual(debug.task.arm_velocity[0], 0.)
                    self.assertEqual(debug.task.velocity, (0.,0.,0.))
                    self.assertIsNone(debug.task.arm_request)
                    self.assertFalse(command('calibration_track_fb')['ok'])
                    self.assertFalse(command('preset_save')['ok'])
                    self.assertFalse(command('tracking_action')['ok'])
                    pump(lambda:request('/api/state').get('calibration_fb_tracking'))
                    debug.vision.hidden = True
                    pump(lambda:debug.task.state == 'TRACK_WAIT' and not peer.jog)
                    debug.vision.hidden = False
                    pump(lambda:debug.task.servo_enabled and peer.jog)
                    self.assertTrue(command('stop')['ok'])
                    pump(lambda:debug._arm_idle() and not peer.jog)
                    self.assertEqual(debug.task.state, 'STOPPED')
                    for _,mid,data in peer.frames[offset:]:
                        if mid == ARM_JOG:
                            yaw, _, lift, _ = JOG_STRUCT.unpack(data)
                            self.assertEqual((yaw, lift), (0.,0.))
                        elif mid == ARM_CMD:
                            self.assertEqual(data[0], 0)
                for category in ('raw', 'stack'):
                    self.assertTrue(command('select',dict(mode='calibration',calibration_type=category))['ok'])
                    result = command('calibration_track_fb')
                    self.assertFalse(result['ok'])
                    self.assertIn('圆环', result['error'])
                self.assertIsNone(peer.error)
                for _,mid,data in peer.frames:
                    if mid == CHASSIS_CMD:
                        self.assertEqual(CHASSIS_STRUCT.unpack(data)[:3],(0.,0.,0.))
                    elif mid == ARM_JOG:
                        self.assertEqual(JOG_STRUCT.unpack(data)[2],0.)
        finally:
            if debug:
                debug.close()
            if executor:
                executor.shutdown()
            if debug:
                debug.destroy_node()
            if bridge:
                bridge.destroy_node()
            peer.close()
            rclpy.shutdown()
