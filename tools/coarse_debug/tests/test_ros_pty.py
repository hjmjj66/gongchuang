"""Opt-in real ROS + HTTP + bridge, using a PTY board and explicit image fixture only."""

from copy import deepcopy
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

from agv_perception.coarse_control import Observation
from coarse_debug.config import FIELDS, area_fields, get_value, read_config


class DebugVisionFixture:
    ready = True
    error = ''

    def __init__(self, board):
        self.board = board
        self.sequence = 0
        self.kind = 'ring'
        self.hide_raw_required = False
        self.frozen = None

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        self.kind = kind

    def latest(self):
        if self.frozen is not None:
            return self.frozen
        self.sequence += 1
        rings = {}
        for n, x in ((1,.06),(2,0.),(3,-.06)):
            u = 100.+(self.board.x-x)*200.
            rings[n] = (u,100.-self.board.y*200.+(u-100.)*math.tan(self.board.yaw))
        if self.kind == 'ring':
            if self.owner.area == 'coarse':
                from test_coarse_ros_pty import coarse_scene
                rings = coarse_scene(self.board)
            if self.owner.area == 'storage':
                from test_coarse_ros_pty import coarse_scene
                rings = coarse_scene(self.board)
            return Observation('ring',self.sequence,time.monotonic(),(201,201),rings)
        if self.owner.area == 'raw':
            from test_raw_ros_pty import raw_scene
            scene = raw_scene(self.board)
            materials = tuple((color,scene[source]) for color,source in ((2,1),(1,3),(3,4))
                              if not (self.hide_raw_required and self.owner.task and
                                      color == 2))
            return Observation('material',self.sequence,time.monotonic(),(201,201),{},materials)
        from test_coarse_ros_pty import coarse_scene
        rings = coarse_scene(self.board)
        materials = tuple((color,rings[ring]) for ring,color in ((1,1),(2,5),(3,6)))
        if self.owner.task and self.owner.task.state == 'SCAN_FIRST_LAYER':
            materials = tuple((color,point) for color,point in materials if abs(point[0]-100.) < 9.)
        return Observation('material',self.sequence,time.monotonic(),(201,201),{},materials)

    def preview(self):
        return b'fixture-jpeg'

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST') == '1', 'opt in to isolated HTTP/ROS/PTY test')
class BrowserRosTests(unittest.TestCase):
    execution = 'simulated'

    def test_http_three_areas_workflow_stop_and_disconnect(self):
        import rclpy
        import sys
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from coarse_debug.node import DebugNode
        from test_coarse_ros_pty import ArmBoardPeer

        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','145')))
        peer=ArmBoardPeer()
        peer.yaw=.08
        bridge=debug=executor=None
        temp=tempfile.TemporaryDirectory(prefix='coarse_debug_pty_')
        try:
            config=read_config(Path(__file__).parents[1]/'config/debug.yaml')
            config.pop('motion_config', None)  # Isolated expanded simulation settings.
            config['debug']['browser_timeout_s']=.8
            from test_coarse_ros_pty import fast_coarse_config
            p=config['coarse']=fast_coarse_config()
            p['angle_alignment']['pid']['kp']=2.
            p['angle_alignment']['max_wz_rad_s']=.3
            from test_storage_ros_pty import fast_storage_config
            storage=config['storage']=fast_storage_config()
            p['base_servo']['tolerance_px'] = 1
            p['gantry_servo']['tolerance_px'] = 1
            storage['base_servo']['tolerance_px'] = 1.
            storage['gantry_servo']['tolerance_px'] = 1.
            from test_raw_control import calibrated_raw_config
            raw = config['raw'] = calibrated_raw_config()
            raw['vision']['target_uv_px']=[100.,100.]
            raw['grasp_gate'].update(tolerance_px=.8,small_yaw_rad_s=.01,small_fb_rpm=.25)
            raw['execution']['feedback_timeout_s']=.5
            for area in ('coarse', 'storage', 'raw'):
                for field in area_fields(area):
                    if field.get('integer'):
                        section = config[area]
                        keys = field['path'].split('.')
                        for key in keys[:-1]: section = section[key]
                        section[keys[-1]] = int(section[keys[-1]])
            # Parameter-save tests must use temporary production destinations too.
            for area in ('coarse', 'storage', 'raw'):
                destination = Path(temp.name)/(area+'.yaml')
                destination.write_text(yaml.safe_dump({area:config[area]}))
                config['debug']['calibration_'+area+'_config'] = str(destination)
            path=Path(temp.name)/'debug.yaml'
            path.write_text(yaml.safe_dump(config,allow_unicode=True))
            bridge=CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=peer.path)])
            debug=DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=DebugVisionFixture(peer))
            debug.vision.owner=debug
            executor=CommandExecutor(bridge._command_sub)
            executor.add_node(bridge)
            executor.add_node(debug)
            peer.thread.start()
            base=f'http://127.0.0.1:{debug.web.http.server_port}'
            client='coarse-debug-browser-test'
            last_heartbeat=0.
            transitions=[]
            raw_speeds=[]

            def request(route, payload=None):
                req=Request(base+route, data=None if payload is None else json.dumps(payload).encode(),
                            headers={} if payload is None else {'Content-Type':'application/json'})
                with urlopen(req,timeout=2.) as response:
                    data=response.read()
                    return data if route=='/preview.jpg' or route in ('/','/app.js','/style.css') else json.loads(data)

            def pump_until(predicate, timeout=6., heartbeat=True):
                nonlocal last_heartbeat
                until=time.monotonic()+timeout
                while time.monotonic()<until:
                    now=time.monotonic()
                    if heartbeat and now-last_heartbeat>.2:
                        request('/api/heartbeat',dict(client=client))
                        last_heartbeat=now
                    executor.spin_once(timeout_sec=.005)
                    if debug.task:
                        if debug.area == 'raw':
                            raw_speeds.append(math.hypot(debug.pose[0].vx, debug.pose[0].vy))
                        phase=(debug.task.state,debug.task.item,debug.task.entered_at)
                        if not transitions or transitions[-1][0]!=phase:
                            transitions.append((phase,time.monotonic(),debug.task.reference,dict(debug.task.placements)))
                    if predicate():
                        return
                self.fail(f'timeout: {debug.task.state if debug.task else None}; {debug.task.reason if debug.task else ""}; '
                          f'healthy={debug._healthy()} graph={debug.graph_ok} peer={peer.error}')

            def command(action, settings=None, op=None, heartbeat=True):
                op=op or uuid.uuid4().hex
                settings = dict(settings or {})
                if action == 'start' and self.execution == 'real':
                    settings.setdefault('execution', self.execution)
                payload=dict(client=client,operation_id=op,action=action,settings=settings)
                request('/api/command',payload)
                def completed():
                    value=debug.web.operations[(client,op)][1]
                    return not value['pending']
                pump_until(completed,heartbeat=heartbeat)
                return debug.web.operations[(client,op)][1],payload

            pump_until(lambda:debug._can_start(debug.vision.latest()))
            self.assertIn('粗加工'.encode(),request('/'))
            self.assertEqual(request('/preview.jpg'),b'fixture-jpeg')
            result,_=command('start',dict(order='112'))
            self.assertFalse(result['ok'])
            self.assertIsNone(debug.task)
            result,_=command('start',dict(order='213',execution='invalid'))
            self.assertFalse(result['ok'])
            self.assertIsNone(debug.task)
            values={f['path']:get_value(config,f['path']) for f in FIELDS}
            values['base_servo.u_pid.kp']=.011
            result,_=command('save',values)
            self.assertTrue(result['ok'])
            self.assertEqual(read_config(path)['coarse']['base_servo']['u_pid']['kp'],.011)
            before=path.read_bytes()
            invalid=deepcopy(values)
            invalid['angle_alignment.timeout_s']=6.
            result,_=command('save',invalid)
            self.assertFalse(result['ok'])
            self.assertEqual(path.read_bytes(),before)
            pump_until(lambda:debug._can_start(debug.vision.latest()))
            result,payload=command('start',dict(order='213'))
            self.assertTrue(result['ok'], result)
            task_id=debug.task.task_id
            request('/api/command',payload)
            executor.spin_once(timeout_sec=.02)
            self.assertEqual(debug.task.task_id,task_id)
            result,_=command('save',values)
            self.assertFalse(result['ok'])
            self.assertEqual(path.read_bytes(),before)
            pump_until(lambda:debug.task.finished,timeout=30.)
            self.assertEqual(debug.task.state,'DONE',debug.task.reason)
            self.assertEqual([(op,ring) for op,_,_,ring in debug.task.confirmed],
                             [('PLACE',2),('PLACE',1),('PLACE',3),('PICK',2),('PICK',1),('PICK',3)])
            self.assertEqual(debug.task.entry_alignment,'ALIGNED')
            self.assertEqual(debug.task.final_alignment,'ALIGNED')
            for index,(phase,_,_,_) in enumerate(transitions[:-1]):
                if self.execution == 'simulated' and phase[0] in ('PLACE','PICK'):
                    self.assertGreaterEqual(transitions[index+1][0][2]-phase[2],1.)
            self.assertEqual(sum(t[0][0] in ('PLACE','PICK') for t in transitions),6)
            self.assertAlmostEqual(debug.pose[0].x, debug.task.reference[0], delta=.0015)
            self.assertAlmostEqual(debug.pose[0].y, debug.task.reference[1], delta=.0015)
            self.assertIn('RETURN_REFERENCE',[t[0][0] for t in transitions])
            coarse_commands=[data[0] for _,mid,data in peer.frames if mid == ARM_CMD]
            self.assertEqual(coarse_commands,[3]*5+[2] if self.execution == 'simulated' else [3,6,6,6,5,5,5,3,2])
            self.assertTrue(any(mid==ARM_JOG for _,mid,_ in peer.frames))
            jogs=[JOG_STRUCT.unpack(data) for _,mid,data in peer.frames if mid == ARM_JOG]
            self.assertTrue(all(jog[2]==0 for jog in jogs))
            storage_frame_start=len(peer.frames)
            pump_until(lambda:request('/api/state')['state']=='DONE')
            self.assertEqual(len(request('/api/state')['confirmed']),6)

            for visit,order in ((1,'213'),(2,'516')):
                result,_=command('select',dict(area='storage',visit=visit))
                self.assertTrue(result['ok'])
                # This harness has no route executor; supply the next area's arrival pose.
                peer.x,peer.y,peer.yaw=0.,0.,0.
                pump_until(lambda:abs(debug.pose[0].x)<.001 and abs(debug.pose[0].y)<.001)
                pump_until(lambda:debug._can_start(debug.vision.latest()))
                origin=(debug.pose[0].x,debug.pose[0].y)
                offset=len(transitions)
                result,_=command('start',dict(area='storage',visit=visit,order=order,layout='156'))
                self.assertTrue(result['ok'])
                result,_=command('select',dict(area='coarse',visit=1))
                self.assertFalse(result['ok'])
                if visit==2:
                    pump_until(lambda:debug.task.state=='SCAN_RESTORE_WORK')
                    self.assertEqual(debug.task.seen_colors,{1,5,6})
                    self.assertFalse(debug.task.confirmed)
                pump_until(lambda:debug.task.finished,timeout=30.)
                self.assertEqual(debug.task.state,'DONE',debug.task.reason)
                expected=[('PLACE',2,2),('PLACE',1,1),('PLACE',3,3)] if visit==1 else [('STACK',5,2),('STACK',1,1),('STACK',6,3)]
                self.assertEqual([(op,c,pos) for op,_,c,pos in debug.task.confirmed],expected)
                self.assertEqual(debug.task.final_alignment,'NOT_RUN')
                self.assertEqual(debug.task.entry_alignment,'ALIGNED' if visit==1 else 'NOT_RUN')
                if visit == 1:
                    self.assertAlmostEqual(debug.pose[0].x, debug.task.reference[0], delta=.0015)
                    self.assertAlmostEqual(debug.pose[0].y, debug.task.reference[1], delta=.0015)
                else:
                    self.assertIsNone(debug.task.reference)
                    self.assertLessEqual(math.dist((debug.pose[0].x,debug.pose[0].y),origin),.003)
                self.assertEqual(debug.task.returned,visit == 1)
                portion=transitions[offset:]
                self.assertEqual('RETURN_REFERENCE' in [t[0][0] for t in portion],visit == 1)
                self.assertNotIn('LOWER',[t[0][0] for t in portion])
                waits=0
                for index,(phase,_,_,_) in enumerate(portion[:-1]):
                    if phase[0] in ('PLACE','STACK'):
                        waits+=1
                        if self.execution == 'simulated':
                            self.assertGreaterEqual(portion[index+1][0][2]-phase[2],1.)
                self.assertEqual(waits,3)

            storage_commands=[data[0] for _,mid,data in peer.frames[storage_frame_start:] if mid==ARM_CMD]
            self.assertEqual(storage_commands,[3,2,3,3,2] if self.execution == 'simulated' else [3,6,6,6,2,3,3,7,7,7,2])
            self.assertTrue(any(mid==ARM_JOG for _,mid,_ in peer.frames[storage_frame_start:]))
            if self.execution == 'simulated':
                self.assertFalse(any(cmd in (6,7) for cmd in storage_commands))
            storage_jogs=[JOG_STRUCT.unpack(data) for _,mid,data in peer.frames[storage_frame_start:] if mid == ARM_JOG]
            self.assertTrue(all(jog[2] == 0 for jog in storage_jogs))
            for failure in ('stop','heartbeat','camera'):
                pump_until(lambda:debug._can_start(debug.vision.latest()))
                result,_=command('start',dict(area='storage',visit=2,order='516',layout='156'))
                self.assertTrue(result['ok'])
                if failure == 'stop':
                    pump_until(lambda:debug.task.servo_enabled)
                    result,_=command('stop')
                    self.assertTrue(result['ok'])
                    self.assertEqual(debug.task.state,'STOPPED')
                elif failure == 'heartbeat':
                    pump_until(lambda:debug.task.finished,timeout=3.,heartbeat=False)
                    self.assertIn('页面失联',debug.task.reason)
                else:
                    pump_until(lambda:debug.task.servo_enabled)
                    debug.vision.frozen=debug.vision.latest()
                    pump_until(lambda:debug.task.finished,timeout=2.)
                    self.assertIn('图像过期',debug.task.reason)
                    debug.vision.frozen=None
                pump_until(lambda:debug._arm_idle() and not peer.jog)
                self.assertFalse(debug.active)
                self.assertEqual(debug.task.arm_velocity,(0.,0.))
                self.assertFalse(debug.task.servo_enabled)
                self.assertEqual(debug.task.velocity,(0.,0.,0.))

            raw_frame_start=len(peer.frames)
            result,_=command('select',dict(area='raw',visit=2))
            self.assertTrue(result['ok'])
            self.assertEqual(debug.visit,1)
            pump_until(lambda:debug._can_start(debug.vision.latest()))
            current_config=read_config(path)
            values={f['path']:get_value(current_config,f['path'],'raw') for f in area_fields('raw')}
            values['initial_follow.duration_s']=.6
            old_other=deepcopy(read_config(path)['storage'])
            result,_=command('save',values)
            self.assertTrue(result['ok'])
            self.assertEqual(read_config(path)['raw']['initial_follow']['duration_s'],.6)
            self.assertEqual(read_config(path)['storage'],old_other)
            result,_=command('start',dict(area='raw',order='112'))
            self.assertFalse(result['ok'])
            origin=(debug.pose[0].x,debug.pose[0].y)
            offset=len(transitions)
            debug.vision.hide_raw_required=True
            pump_until(lambda:debug._can_start(debug.vision.latest()))
            result,_=command('start',dict(area='raw',order='213'))
            self.assertTrue(result['ok'])
            pump_until(lambda:debug.task.state=='WAIT_REQUIRED')
            self.assertIsNone(debug.task.tracked_color)
            self.assertFalse(debug.task.confirmed)
            self.assertIsNone(debug.task.reference)
            debug.vision.hide_raw_required=False
            pump_until(lambda:debug.task.tracked_color==2)
            self.assertIsNone(debug.task.arm_request)
            result,_=command('select',dict(area='coarse'))
            self.assertFalse(result['ok'])
            result,_=command('save',values)
            self.assertFalse(result['ok'])
            pump_until(lambda:debug.task.finished,timeout=30.)
            self.assertEqual(debug.task.state,'DONE',debug.task.reason)
            self.assertEqual(debug.task.confirmed,[('PICK',1,2,0),('PICK',2,1,0),('PICK',3,3,0)])
            self.assertEqual(debug.task.entry_alignment,'NOT_RUN')
            self.assertEqual(debug.task.final_alignment,'NOT_RUN')
            commands=[data[0] for _,mid,data in peer.frames[raw_frame_start:] if mid == ARM_CMD]
            self.assertEqual(commands,[3,3,3,3,2] if self.execution == 'simulated' else [3,4,4,4,2])
            self.assertTrue(any(mid == ARM_JOG for _,mid,_ in peer.frames))
            self.assertLessEqual(math.dist((debug.pose[0].x,debug.pose[0].y),origin),.003)
            portion=transitions[offset:]
            waits=0
            for index,(phase,_,_,_) in enumerate(portion[:-1]):
                if phase[0]=='PICK':
                    waits+=1
                    if self.execution == 'simulated':
                        self.assertGreaterEqual(portion[index+1][0][2]-phase[2],1.)
            self.assertEqual(waits,3)
            pump_until(lambda:request('/api/state')['state']=='DONE')
            snapshot=request('/api/state')
            self.assertEqual(snapshot['task_execution'],self.execution)
            self.assertEqual(snapshot['area'],'raw')
            self.assertEqual(snapshot['order'],[2,1,3])
            self.assertEqual(len(snapshot['confirmed']),3)
            self.assertEqual(snapshot['placements'],{})

            result,_=command('start',dict(area='raw',order='123'))
            self.assertTrue(result['ok'])
            pump_until(lambda:any(abs(v)>1e-5 for v in (debug.task.arm_velocity if debug.area == 'raw' else debug.task.velocity)))
            result,_=command('stop')
            self.assertTrue(result['ok'])
            self.assertEqual(debug.task.state,'STOPPED')
            pump_until(lambda:all(abs(v)<1e-9 for v in peer.command[:3]))
            pump_until(lambda:debug._arm_idle() and not peer.jog)
            result,_=command('start',dict(area='raw',order='321'))
            self.assertTrue(result['ok'])
            pump_until(lambda:debug.task.finished,timeout=3.,heartbeat=False)
            self.assertEqual(debug.task.state,'ERROR')
            self.assertIn('页面失联',debug.task.reason)
            pump_until(lambda:all(abs(v)<1e-9 for v in peer.command[:3]),heartbeat=False)
            pump_until(lambda:debug._arm_idle() and not peer.jog)
            result,_=command('select',dict(area='coarse',visit=1))
            self.assertTrue(result['ok'])
            pump_until(lambda:debug._can_start(debug.vision.latest()))

            result,_=command('start',dict(order='321'))
            self.assertTrue(result['ok'])
            pump_until(lambda:any(abs(v)>1e-5 for v in (*debug.task.velocity,*debug.task.arm_velocity)))
            result,_=command('stop')
            self.assertTrue(result['ok'])
            self.assertEqual(debug.task.state,'STOPPED')
            pump_until(lambda:all(abs(v)<1e-9 for v in peer.command[:3]))
            pump_until(lambda:debug._arm_idle() and not peer.jog)

            result,_=command('start',dict(order='123'))
            self.assertTrue(result['ok'])
            pump_until(lambda:debug.task.finished,timeout=3.,heartbeat=False)
            self.assertEqual(debug.task.state,'ERROR')
            self.assertIn('页面失联',debug.task.reason)
            pump_until(lambda:all(abs(v)<1e-9 for v in peer.command[:3]),heartbeat=False)
            pump_until(lambda:debug._arm_idle() and not peer.jog)
            pump_until(lambda:debug._can_start(debug.vision.latest()))
            result,_=command('start',dict(order='231'))
            self.assertTrue(result['ok'])
            pump_until(lambda:debug.task.servo_enabled)
            debug.vision.frozen=debug.vision.latest()
            pump_until(lambda:debug.task.finished,timeout=2.)
            self.assertEqual(debug.task.state,'ERROR')
            self.assertIn('图像过期',debug.task.reason)
            pump_until(lambda:debug._arm_idle() and not peer.jog)
            self.assertTrue(any(mid==CHASSIS_CMD and any(abs(v)>0 for v in CHASSIS_STRUCT.unpack(payload)[:3])
                                for _,mid,payload in peer.frames))
            self.assertFalse(any(mid == ARM_CMD and data[0] == 9 for _,mid,data in peer.frames))
            self.assertFalse(peer.jog)
            self.assertIsNone(peer.error)
        finally:
            if debug:
                debug.close()
                if executor:
                    executor.remove_node(debug)
                debug.destroy_node()
            if bridge:
                if executor:
                    executor.remove_node(bridge)
                bridge.destroy_node()
            if executor:
                executor.shutdown()
            peer.close()
            temp.cleanup()
            rclpy.shutdown()
            sys.setswitchinterval(switch_interval)


class RealBrowserRosTests(BrowserRosTests):
    execution = 'real'


if __name__=='__main__':
    unittest.main()
