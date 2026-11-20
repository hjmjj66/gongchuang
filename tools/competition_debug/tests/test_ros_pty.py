"""Actual HTTP, planner, bridge, batch host, QR decoder and screen; only PTYs/images."""

from agv_perception.work_config import read_work_config

import json
import math
import os
import select
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
import uuid

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[3]


class SoftwareCamera:
    text = '156+231+516+312'

    def __init__(self, device, width, height, fps, timestamp, logger, *args, **kwargs):
        self.timestamp=timestamp
        self.sequence=0
        self.closed=False

    def start(self):pass

    def latest(self):
        if self.closed:return None
        self.sequence+=1
        qr=cv2.QRCodeEncoder_create().encode(self.text)
        qr=cv2.copyMakeBorder(qr,5,5,5,5,cv2.BORDER_CONSTANT,value=255)
        pixels=cv2.cvtColor(cv2.resize(qr,(480,480),interpolation=cv2.INTER_NEAREST),cv2.COLOR_GRAY2BGR)
        return self.sequence,pixels,self.timestamp(),time.monotonic()

    def close(self):self.closed=True


class SceneVision:
    error=''
    camera=None

    def __init__(self, peer):
        self.peer=peer;self.kind='ring';self.sequence=0;self.task_id='';self.geometry_target=0
        self.missing=False

    def open(self):self.camera=self
    def close(self):self.camera=None
    def diagnostics(self):return dict(open=self.camera is not None, source='software scene')
    def preview(self):
        ok,data=cv2.imencode('.jpg',cv2.UMat(20,20,cv2.CV_8UC3).get())
        return data.tobytes() if ok else None
    def request(self,kind,*,geometry_target=0,largest_per_color=False):
        self.kind=kind;self.geometry_target=geometry_target
        self.largest_per_color=largest_per_color

    def latest(self):
        from agv_perception.work_control import Observation
        from test_coarse_ros_pty import coarse_scene
        from test_raw_ros_pty import raw_scene
        task=self.owner.task
        if task.task_id!=self.task_id:
            self.task_id=task.task_id;self.origin=(self.peer.x,self.peer.y,self.peer.yaw)
        x,y,yaw=self.origin
        dx,dy=self.peer.x-x,self.peer.y-y
        board=SimpleNamespace(x=math.cos(yaw)*dx+math.sin(yaw)*dy,y=-math.sin(yaw)*dx+math.cos(yaw)*dy,
            yaw=self.peer.yaw-yaw,arm_yaw=self.peer.arm_yaw,arm_fb=self.peer.arm_fb)
        self.sequence+=1
        rings=coarse_scene(board)
        if self.owner.area=='raw':
            points=raw_scene(board)
            materials=tuple((color,points[source]) for color,source in ((1,1),(5,3),(6,4)))
        else:
            materials=tuple((color,rings[ring]) for color,ring in ((1,2),(5,3),(6,1)))
        if self.missing:materials=()
        return Observation(self.kind,self.sequence,time.monotonic(),(201,201),rings,materials,
                           geometry_target=self.geometry_target)


@unittest.skipUnless(os.environ.get('AGV_COMPETITION_DEBUG_ROS_TEST')=='1','opt in to isolated software ROS/PTY test')
class FullRosTests(unittest.TestCase):
    def test_complete_modes_real_and_dwell_stop_reset_invalid_qr(self):
        import sys
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_planning.route_planner_node import RoutePlannerNode
        from agv_planning.planner import EDGES
        from competition_debug.node import CompetitionNode
        from test_coarse_ros_pty import ArmBoardPeer, fast_coarse_config
        from test_storage_ros_pty import fast_storage_config
        from test_raw_control import calibrated_raw_config
        from route_debug.node import PREFIX
        interval=sys.getswitchinterval();sys.setswitchinterval(.001)
        rclpy.init()
        peer=ArmBoardPeer();peer.thread.start()
        screen_master,screen_slave=os.openpty()
        nodes=[];executor=None;debug=None
        try:
            with tempfile.TemporaryDirectory(prefix='competition_debug_') as temp, patch('agv_perception.qr_node.CameraStream',SoftwareCamera):
                folder=Path(temp)
                config={a:read_work_config((ROOT/'ros2_ws/src/agv_perception/config'/(a+'.yaml')))[a]
                        for a in ('raw','coarse','storage')}
                formal={a:config[a]['vision'] for a in config}
                config['coarse']=fast_coarse_config();config['storage']=fast_storage_config()
                config['raw']=calibrated_raw_config()
                raw=config['raw']
                raw['vision']['target_uv_px']=[100.,100.]
                raw['grasp_gate'].update(tolerance_px=.8,small_yaw_rad_s=.01,small_fb_rpm=.25)
                paths={}
                for area,p in config.items():
                    p['execution']['feedback_timeout_s']=.5
                    for key in ('ring_config','material_config'):
                        if key in formal[area]:p['vision'][key]=formal[area][key]
                    paths[area+'_config']=str(folder/(area+'.yaml'))
                    Path(paths[area+'_config']).write_text(yaml.safe_dump({area:p}))
                mission_file=folder/'mission.yaml'
                mission_file.write_text(yaml.safe_dump({'mission_node':{'ros__parameters':dict(use_lidar=False,parking_offset_mm=20)}}))
                params=dict(default_edge_length_mm=80,translation_pid_kp=8.,translation_pid_kd=0.,
                    angle_pid_kp=8.,angle_pid_kd=0.,angle_max_speed_deg_s=160.,turn_arrival_tolerance_deg=1.,
                    debug_allow_any_pose=True,debug_publish_progress=True)
                params.update({f'edge_length_mm.{min(a,b)}_{max(a,b)}':80 for a,b in EDGES})
                params.update({f'work_heading.node_{n}':h for n,h in ((2,'RIGHT'),(4,'LEFT'),(6,'RIGHT'),(8,'DOWN'))})
                params.update({key:PREFIX+'/'+value for key,value in (('plan_service','plan'),('route_topic','path'),
                    ('control_topic','control'),('execution_status_topic','status'),('blocked_edges_topic','map'),('debug_progress_topic','progress'))})
                planner_file=folder/'planner.yaml';planner_file.write_text(yaml.safe_dump({'route_planner_node':{'ros__parameters':params}}))
                planner=RoutePlannerNode(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()],
                    cli_args=['--ros-args',*sum((['-r',f'/route/{key}:={PREFIX}/{key}'] for key in ('get_runtime','stop','reset_context')),[]),
                              '-r','/mission/heartbeat:='+PREFIX+'/heartbeat'])
                nodes.append(planner)
                bridge=CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=peer.path)])
                nodes.append(bridge)
                args=SimpleNamespace(**paths,planner_config=str(planner_file),mission_config=str(mission_file),
                    cboard_config=str(ROOT/'ros2_ws/src/agv_cboard/config/cboard.yaml'),
                    qr_config=str(ROOT/'ros2_ws/src/agv_perception/config/qr.yaml'),
                    screen_config=str(ROOT/'ros2_ws/src/agv_screen/config/screen.yaml'),screen_port=os.ttyname(screen_slave),
                    debug_config=str(ROOT/'tools/route_debug/config/debug.yaml'),work_config=paths['coarse_config'],host='127.0.0.1',port=0)
                debug=CompetitionNode(args);nodes.append(debug)
                vision=SceneVision(peer);vision.owner=debug.work;debug.work.vision=vision
                executor=CommandExecutor(bridge._command_sub)
                for node in nodes:executor.add_node(node)
                base=f'http://127.0.0.1:{debug.web.http.server_port}'
                client='full-run-pty-browser'
                last_heartbeat=0.

                def request(route,payload=None):
                    req=Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=2) as response:return json.load(response)

                def pump(predicate,timeout=120,allow_fault=False):
                    nonlocal last_heartbeat
                    deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        if time.monotonic()-last_heartbeat>.2:
                            request('/api/heartbeat',dict(client=client));last_heartbeat=time.monotonic()
                        executor.spin_once(timeout_sec=.003)
                        if predicate():return
                        if not allow_fault:
                            self.assertNotEqual(debug.session.state,'FAULT',debug.session.reason)
                    self.fail(f'timeout: {debug.session.state} {debug.session.reason}; task={debug.work.task and debug.work.task.state}; diagnostic={debug.work.task and debug.work.task.diagnostic}; peer={peer.error}')

                def command(action,settings=None):
                    op=uuid.uuid4().hex
                    request('/api/command',dict(client=client,action=action,settings=settings or {},operation_id=op))
                    pump(lambda:not debug.web.operations[(client,op)][1]['pending'],allow_fault=debug.session.state=='FAULT')
                    result=debug.web.operations[(client,op)][1]
                    self.assertTrue(result['ok'],str(result)+f'; pending={debug.session.pending}; phase={debug.session.phase}; '
                        f'work={debug.work.task and debug.work.task.state}; future={debug.work.future}; '
                        f'unknown={debug.work.unknown}; abort={debug.work.abort_id}; arm={debug.work.arm}; '
                        f'arm_age={time.monotonic()-debug.work.arm_at}')

                pump(lambda:debug.session.health()=='' and debug.services_ready())
                arm_begin=len(peer.arm_actions)
                command('camera_open')
                pump(lambda:request('/api/state').get('camera_manual'))
                self.assertEqual(debug.session.state,'IDLE')
                with urlopen(base+'/preview.jpg',timeout=2) as response:
                    self.assertEqual(response.headers['Content-Type'],'image/jpeg')
                    self.assertTrue(response.read())
                self.assertEqual(len(peer.arm_actions),arm_begin)
                command('camera_close')
                self.assertIsNone(vision.camera)
                command('camera_open')
                for mode,real in (('web',False),('screen',True)):
                    begin=len(peer.arm_actions)
                    command('start',dict(mode=mode,start=1,goal=3,real_actions=dict.fromkeys(config,real)))
                    self.assertFalse(debug.idle_preview)
                    if mode=='screen':
                        self.assertEqual(debug.session.state,'SCREEN_SETUP')
                        # Both OK buttons only select; the separate START begins initialization.
                        os.write(screen_master,b'ROUTE,1\r\nCFG,1,3\r\n')
                        pump(lambda:debug.session.screen_zones==(1,3))
                        self.assertEqual(debug.session.state,'SCREEN_SETUP')
                        configured_at=time.monotonic()
                        os.write(screen_master,b'START\r\nSTART\r\n')
                        pump(lambda:debug.session.state=='NAVIGATING')
                        self.assertGreaterEqual(time.monotonic()-configured_at,2.)
                    pump(lambda:debug.session.state=='FINISHED')
                    pump(lambda:request('/api/state')['state']=='FINISHED')
                    snapshot=request('/api/state')
                    self.assertEqual(len(snapshot['results']),24)
                    self.assertEqual(snapshot['qr_result'],SoftwareCamera.text)
                    if mode=='screen':
                        pump(lambda:not debug.screen.outgoing)
                        display=b''
                        while select.select([screen_master],[],[],0)[0]:
                            display+=os.read(screen_master,8192)
                        self.assertIn(b'page result\xff\xff\xfft4.txt="12"\xff\xff\xfft5.txt="12"\xff\xff\xff',display)
                    self.assertTrue(all(row['outcome']==('REAL_COMPLETED' if real else 'DWELL_COMPLETED') for row in snapshot['results']))
                    self.assertGreater(len(snapshot['log']),80)
                    self.assertIsNone(vision.camera)
                    actions=[r[0] for r in peer.arm_actions[begin:]]
                    if real:
                        self.assertEqual({cmd:actions.count(cmd) for cmd in (4,5,6,7,8)},{4:6,5:6,6:9,7:3,8:0})
                    else:self.assertFalse(set(actions)&{4,5,6,7,8})
                    pump(lambda:debug.work_released())
                    command('reset')
                    self.assertEqual(debug.session.state,'IDLE')
                SoftwareCamera.text='invalid-task'
                command('start',dict(mode='web',start=1,goal=1))
                pump(lambda:debug.session.state=='SCANNING' and bool(debug.qr_wait))
                self.assertIsNone(debug.session.qr_result)
                command('stop')
                pump(lambda:debug.session.state=='STOPPED')
                self.assertIsNone(debug.qr)
                command('reset')
                self.assertFalse(debug.session.locked)
                SoftwareCamera.text='156+231+516+312'
                command('start',dict(mode='web',start=1,goal=1,real_actions=dict.fromkeys(config,False)))
                pump(lambda:debug.session.state=='WORKING' and debug.work.task.state=='TRACK_REQUIRED')
                with urlopen(base+'/preview.jpg',timeout=2) as response:
                    self.assertEqual(response.headers.get_content_type(),'image/jpeg')
                command('stop')
                pump(lambda:debug.session.state=='STOPPED')
                self.assertEqual(peer.arm_actions[-1][0],0)
                self.assertTrue(debug.work_released())
                self.assertIsNone(vision.camera)
                command('reset')
                command('start',dict(mode='screen'))
                debug.session.owner_at=time.monotonic()-9.
                debug.session.tick()
                pump(lambda:debug.session.state=='FAULT')
                self.assertIn('失联',debug.session.reason)
                self.assertFalse(debug.screen.accepting)
                self.assertTrue(any('故障执行上下文' in row['message'] for row in debug.session.log))
                command('reset')
                self.assertIsNone(peer.error)
        finally:
            SoftwareCamera.text='156+231+516+312'
            if debug:debug.close()
            if executor:executor.shutdown()
            for node in reversed(nodes):node.destroy_node()
            peer.close();os.close(screen_master);os.close(screen_slave)
            rclpy.shutdown();sys.setswitchinterval(interval)
