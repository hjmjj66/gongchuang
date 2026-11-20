"""HTTP calibration, persistence and job prepositioning over ROS and a software PTY."""

from copy import deepcopy
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


class PoseVision:
    ready, error, kind, hidden = True, '', 'ring', False

    def __init__(self, peer):
        self.peer, self.sequence = peer, 0

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        self.kind = kind

    def latest(self):
        self.sequence += 1
        b = self.peer
        import math
        rings = {}
        for n,x in ((1,.06),(2,0.),(3,-.06)):
            u=100.+(b.x-x)*200.+b.arm_yaw*100.
            rings[n]=(u,100.-b.y*200.+(u-100.)*math.tan(b.yaw)+(b.arm_fb-(3. if n!=2 else 0.))*5.)
        return Observation(self.kind,self.sequence,time.monotonic(),(201,201),
                           {} if self.hidden else rings,tuple((n,p) for n,p in rings.items()))

    def preview(self):
        return b'pose-fixture'

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST')=='1','opt in to software HTTP/ROS/PTY test')
class PoseCalibrationRosTests(unittest.TestCase):
    def test_shared_calibration_saves_once_and_prepositions_all_area_jobs(self):
        import sys
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, JOG_STRUCT
        from coarse_debug.config import area_fields, read_config
        from agv_perception.gantry_presets import DEFAULTS
        from coarse_debug.node import DebugNode
        from test_coarse_ros_pty import ArmBoardPeer, fast_coarse_config
        from test_storage_ros_pty import fast_storage_config

        switch_interval=sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','187')))
        peer=ArmBoardPeer()
        bridge=debug=executor=None
        peer.thread.start()
        root=Path(__file__).resolve().parents[3]
        try:
            with tempfile.TemporaryDirectory(prefix='preset_web_') as folder:
                folder=Path(folder)
                config=read_config(Path(__file__).resolve().parents[1]/'config/debug.yaml')
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                config['coarse']=fast_coarse_config()
                config['storage']=fast_storage_config()
                config['debug']['browser_timeout_s']=.8
                for area in ('coarse','storage'):
                    p=config[area]
                    for field in area_fields(area):
                        if field.get('integer'):
                            parts=field['path'].split('.')
                            section=p
                            for key in parts[:-1]:
                                section=section[key]
                            section[parts[-1]]=max(1 if field['positive'] else 0, int(section[parts[-1]]))
                    p.setdefault('gantry_presets',deepcopy(DEFAULTS))
                    p['gantry_motion'].update(length_pid=dict(kp=4.*60/3200,ki=0.,kd=0.),yaw_pid=dict(kp=4.,ki=0.,kd=0.),max_fb_rpm=20.,max_yaw_rad_s=.4,
                                               length_tolerance_pulses=5.,angle_tolerance_rad=.005)
                    path=folder/f'{area}.yaml'
                    path.write_text(yaml.safe_dump({area:deepcopy(p)},allow_unicode=True))
                    config['debug'][f'calibration_{area}_config']=str(path)
                mission=folder/'mission.yaml'
                mission.write_text('mission_node:\n  ros__parameters:\n    camera_car_offset: 0.0\n')
                config['debug']['mission_config']=str(mission)
                path=folder/'debug.yaml'
                path.write_text(yaml.safe_dump(config,allow_unicode=True))
                bridge=CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=peer.path)])
                debug=DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=PoseVision(peer))
                executor=CommandExecutor(bridge._command_sub)
                executor.add_node(bridge)
                executor.add_node(debug)
                base=f'http://127.0.0.1:{debug.web.http.server_port}'
                client,last_heartbeat='preset-browser',0.

                def request(route,payload=None):
                    req=Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=2.) as response:
                        return json.loads(response.read())

                def pump(predicate,timeout=12.,heartbeat=True,allow_error=False):
                    nonlocal last_heartbeat
                    until=time.monotonic()+timeout
                    while time.monotonic()<until:
                        if heartbeat and time.monotonic()-last_heartbeat>.15:
                            request('/api/heartbeat',dict(client=client))
                            last_heartbeat=time.monotonic()
                        executor.spin_once(timeout_sec=.005)
                        if predicate():
                            return
                        if not allow_error and debug.task and debug.task.state=='ERROR':
                            self.fail(debug.task.reason)
                    self.fail(f'timeout {debug.task and debug.task.state}: {debug.task and debug.task.diagnostic}; peer={peer.error}')

                def command(action,settings=None,who=client):
                    op=uuid.uuid4().hex
                    request('/api/command',dict(client=who,operation_id=op,action=action,settings=settings or {}))
                    pump(lambda:not debug.web.operations[(who,op)][1]['pending'],allow_error=action=='preset_save')
                    return debug.web.operations[(who,op)][1]

                pump(lambda:debug._can_start(debug.vision.latest()))
                self.assertTrue(command('select',dict(mode='calibration',calibration_type='shared_pose'))['ok'])
                self.assertTrue(command('preset_save',dict(target=3))['ok'])
                self.assertTrue(debug.task.saved)
                self.assertIsNone(debug.reference_ready)
                pump(lambda:debug._can_start(debug.vision.latest()))
                self.assertTrue(command('preset_track',dict(target=3))['ok'])
                pump(lambda:debug.task.servo_enabled)
                self.assertTrue(command('preset_track',dict(target=3))['ok'])
                self.assertTrue(command('stop')['ok'])
                pump(lambda:debug._can_start(debug.vision.latest()))
                peer.x,peer.y,peer.yaw=.012,-.01,.04
                offset=len(peer.frames)
                self.assertTrue(command('preset_adjust')['ok'])
                pump(lambda:debug.task.finished)
                self.assertEqual(debug.task.state,'DONE')
                self.assertEqual(debug.task.entry_alignment,'ALIGNED')
                self.assertIsNotNone(debug.reference_ready)
                self.assertEqual([data[0] for _,mid,data in peer.frames[offset:] if mid==ARM_CMD and data[0]!=0],[3])
                for ring in (3,1):
                    pump(lambda:debug._can_start(debug.vision.latest()))
                    result=command('preset_track',dict(target=ring))
                    self.assertTrue(result['ok'],result)
                    pump(lambda:debug.task.servo_enabled)
                    self.assertFalse(command('tracking_action')['ok'])
                    self.assertTrue(command('preset_save',dict(target=ring))['ok'])
                    pump(lambda:debug.task.finished)
                    self.assertTrue(debug.task.saved)
                    point=debug.task.sample
                    self.assertIsInstance(point['fb_pulses'], int)
                    for area in ('coarse','storage'):
                        self.assertEqual(read_config(path)[area]['gantry_presets']['positions'][str(ring)],point)
                        formal=yaml.safe_load((folder/f'{area}.yaml').read_text())
                        self.assertEqual(formal[area]['gantry_presets']['positions'][str(ring)],point)
                    pump(lambda:request('/api/state')['preset_saved'])
                    snapshot=request('/api/state')
                    self.assertEqual(snapshot['calibration_type'],'shared_pose')
                    self.assertEqual(snapshot['gantry_presets']['coarse'],snapshot['gantry_presets']['storage'])
                self.assertEqual([data[0] for _,mid,data in peer.frames[offset:] if mid==ARM_CMD and data[0]!=0],[3])
                pump(lambda:debug._can_start(debug.vision.latest()))
                for area in ('coarse','storage'):
                    # Saved values now drive normal jobs; each move must stop before fresh visual frames.
                    self.assertTrue(command('select',dict(mode='task',area=area,visit=1))['ok'])
                    pump(lambda:debug._can_start(debug.vision.latest()))
                    seen=set()
                    self.assertTrue(command('start',dict(area=area,visit=1,order='312'))['ok'])
                    def job_done():
                        seen.add(debug.task.state)
                        return debug.task.finished
                    pump(job_done,timeout=35.)
                    self.assertEqual(debug.task.state,'DONE')
                    self.assertTrue({'PREPOSITION','STOP_PREPOSITION','ALIGN_RING'}<=seen,seen)
                # Second-visit storage reuses the exact same positions and keeps the chassis still.
                self.assertTrue(command('select',dict(mode='task',area='storage',visit=2))['ok'])
                pump(lambda:debug._can_start(debug.vision.latest()))
                self.assertTrue(command('start',dict(area='storage',visit=2,order='312',layout='123'))['ok'])
                seen=set()
                start_xy=peer.x,peer.y
                def stack_done():
                    seen.add(debug.task.state)
                    self.assertEqual(debug.task.velocity,(0.,0.,0.))
                    return debug.task.finished
                pump(stack_done,timeout=35.)
                self.assertEqual(debug.task.state,'DONE')
                self.assertTrue({'PREPOSITION','STOP_PREPOSITION','ALIGN_MATERIAL'}<=seen,seen)
                self.assertEqual((peer.x,peer.y),start_xy)
                self.assertTrue(command('select',dict(mode='calibration',calibration_type='shared_pose'))['ok'])
                pump(lambda:debug._can_start(debug.vision.latest()))
                self.assertTrue(command('preset_adjust')['ok'])
                pump(lambda:debug.task.finished)
                debug.parameters['gantry_servo']['tolerance_px'] = 1e-9
                self.assertTrue(command('preset_track',dict(target=3))['ok'])
                pump(lambda:debug.task.servo_enabled and peer.jog)
                self.assertEqual(debug.task.qualified_frames, 0)
                self.assertTrue(command('stop')['ok'])
                self.assertEqual(debug.task.state,'STOPPED')
                self.assertIsNone(debug.reference_ready)
                self.assertEqual(debug.task.velocity,(0.,0.,0.))
                self.assertEqual(debug.task.arm_velocity,(0.,0.))
                pump(lambda:debug._arm_idle() and not peer.jog)

                debug.vision.hidden = True
                self.assertTrue(command('preset_save',dict(target=3),who='another-browser')['ok'])
                pump(lambda:debug.task.finished)
                self.assertTrue(debug.task.saved)
                self.assertEqual(debug.task.state, 'DONE')
                point = debug.task.sample
                for area in ('coarse','storage'):
                    self.assertEqual(read_config(path)[area]['gantry_presets']['positions']['3'], point)
                    formal = yaml.safe_load((folder/f'{area}.yaml').read_text())
                    self.assertEqual(formal[area]['gantry_presets']['positions']['3'], point)
                for _,mid,data in peer.frames:
                    if mid==ARM_JOG:
                        self.assertEqual(JOG_STRUCT.unpack(data)[2],0.)
                # A real competing ROS writer must name the topic and node in the error.
                from agv_interfaces.msg import ControlCommand
                pump(lambda:debug._can_start(debug.vision.latest()))
                self.assertTrue(command('preset_track',dict(target=3))['ok'])
                conflict = rclpy.create_node('calibration_conflicting_writer')
                writer = conflict.create_publisher(ControlCommand, '/C/controll', 1)
                try:
                    pump(lambda:debug.task.finished and not debug.graph_ok)
                    self.assertEqual(debug.task.state, 'ERROR')
                    self.assertIn('/C/controll', debug.task.reason)
                    self.assertIn('calibration_conflicting_writer', debug.task.reason)
                    self.assertIn('实际2个', debug.task.reason)
                    # Saving does not need exclusive control, an image or a prior tracking owner.
                    self.assertTrue(command('preset_save',dict(target=1),who='another-browser')['ok'])
                    self.assertTrue(debug.task.saved)
                    self.assertFalse(debug.graph_ok)
                    self.assertEqual(read_config(path)['coarse']['gantry_presets']['positions']['1'], debug.task.sample)
                finally:
                    conflict.destroy_publisher(writer)
                    conflict.destroy_node()
                pump(lambda:debug._healthy())
                # Invalid or stale actual positions must never become fake zero presets.
                position_flags, board_at = debug.board.pos_flags, debug.board_at
                try:
                    debug.board.pos_flags = 0
                    with self.assertRaisesRegex(ValueError, '位置反馈无效或过期'):
                        debug._command('preset_save',dict(target=1),client)
                    debug.board.pos_flags = position_flags
                    debug.board_at = time.monotonic()-10.
                    with self.assertRaisesRegex(ValueError, '位置反馈无效或过期'):
                        debug._command('preset_save',dict(target=1),client)
                finally:
                    debug.board.pos_flags, debug.board_at = position_flags, board_at
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
            peer.close()
            rclpy.shutdown()
            sys.setswitchinterval(switch_interval)
