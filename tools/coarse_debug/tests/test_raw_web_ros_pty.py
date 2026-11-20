"""Opt-in raw HTTP/ROS/PTY integration, independent of concurrent coarse redesign."""
from copy import deepcopy
import json
import os
import sys
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen
import uuid
import yaml

from agv_perception.work_control import Observation
from coarse_debug.config import area_fields, get_value, read_config
from test_coarse_ros_pty import ArmBoardPeer
from test_raw_control import calibrated_raw_config
from test_raw_ros_pty import raw_scene


class RawWebScene:
    ready = True
    error = ''

    def __init__(self, board):
        self.board = board
        self.sequence = 0
        self.packet = None
        self.at = 0.
        self.hidden = False

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        pass

    def latest(self):
        now=time.monotonic()
        if now-self.at < .035:
            return self.packet
        self.at=now
        self.sequence+=1
        scene=raw_scene(self.board)
        materials=tuple((color,scene[source]) for color,source in ((2,1),(1,3),(3,4))
                        if not (self.hidden and color == 2))
        self.packet=Observation('material',self.sequence,now,(201,201),{},materials)
        return self.packet

    def preview(self):
        return b'explicit-software-scene'

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST')=='1','opt in to isolated raw HTTP/ROS/PTY')
class RawBrowserRosTests(unittest.TestCase):
    def test_real_work_gantry_dwell_restore_stow_return_and_independent_save(self):
        self.run_scenario('complete')

    def test_loss_recovery_wait_cancel_and_browser_disconnect_stop_both(self):
        self.run_scenario('loss_stop')

    def run_scenario(self,scenario):
        import rclpy
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from coarse_debug.node import DebugNode

        switch_interval=sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','183'))+(scenario=='loss_stop'))
        board=ArmBoardPeer()
        nodes=[]
        executor=None
        board.thread.start()
        with tempfile.TemporaryDirectory(prefix='raw_web_pty_') as folder:
            try:
                config=read_config(Path(__file__).parents[1]/'config/debug.yaml')
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                config['raw']=calibrated_raw_config()
                config['raw']['motion_prediction']['enabled']=True
                config['raw']['vision']['target_uv_px']=[100.,100.]
                config['raw']['execution']['feedback_timeout_s']=.5
                config['debug']['browser_timeout_s']=.8
                for area in ('raw','coarse','storage'):
                    destination=Path(folder)/(area+'.yaml')
                    destination.write_text(yaml.safe_dump({area:config[area]}))
                    config['debug']['calibration_'+area+'_config']=str(destination)
                path=Path(folder)/'debug.yaml'
                path.write_text(yaml.safe_dump(config,allow_unicode=True))
                bridge=CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=board.path)])
                nodes.append(bridge)
                scene=RawWebScene(board)
                debug=DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=scene)
                nodes.append(debug)
                executor=CommandExecutor(bridge._command_sub)
                for node in nodes:
                    executor.add_node(node)
                base=f'http://127.0.0.1:{debug.web.http.server_port}'
                client='raw-pty-browser'
                phases=[]
                last_heartbeat=0.

                def request(route,payload=None):
                    req=Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                headers={} if payload is None else {'Content-Type':'application/json'})
                    with urlopen(req,timeout=2.) as response:
                        return json.loads(response.read())

                def pump_until(predicate,timeout=8.,heartbeat=True):
                    nonlocal last_heartbeat
                    until=time.monotonic()+timeout
                    while time.monotonic()<until:
                        now=time.monotonic()
                        if heartbeat and now-last_heartbeat>.2:
                            request('/api/heartbeat',dict(client=client))
                            last_heartbeat=now
                        executor.spin_once(timeout_sec=.005)
                        if debug.task:
                            if debug.task.state in (*debug.task.ARM_STATES, 'WAIT_REQUIRED', 'TRACK_REQUIRED'):
                                self.assertEqual(debug.task.velocity,(0.,0.,0.))
                            phase=(debug.task.state,debug.task.item,debug.task.entered_at)
                            if not phases or phase != phases[-1]:
                                phases.append(phase)
                        if predicate():
                            return
                    self.fail(f'{scenario}: {debug.task.state if debug.task else None}; '
                              f'{debug.task.reason if debug.task else ""}; '
                              f'arm={bridge._arm.state}:{bridge._arm.reason}; board={board.error}')

                def command(action,settings=None):
                    op=uuid.uuid4().hex
                    request('/api/command',dict(client=client,operation_id=op,action=action,settings=settings or {}))
                    pump_until(lambda: not debug.web.operations[(client,op)][1]['pending'])
                    result=debug.web.operations[(client,op)][1]
                    self.assertTrue(result['ok'],result)

                pump_until(lambda: debug._healthy())
                command('select',dict(area='raw',visit=1))
                pump_until(lambda: debug._can_start(scene.latest()))
                values={f['path']:get_value(config,f['path'],'raw') for f in area_fields('raw')}
                values['base_motion.distance_mm']=18.
                values['grasp_gate.qualified_frames']=6
                other=deepcopy(config['storage'])
                command('save',values)
                self.assertEqual(read_config(path)['storage'],other)
                self.assertEqual(read_config(path)['raw']['grasp_gate']['qualified_frames'],6)
                self.assertEqual(yaml.safe_load((Path(folder)/'raw.yaml').read_text())['raw']['grasp_gate']['qualified_frames'],6)
                board.x=.03
                pump_until(lambda: debug.pose[0].x>.02 and debug._can_start(scene.latest()))
                command('start',dict(area='raw',order='213'))
                self.assertEqual(debug.task.p['grasp_gate']['qualified_frames'],6)
                pump_until(lambda: debug.task.state=='TRACK_REQUIRED')
                self.assertTrue(debug.task.base_locked)
                self.assertEqual(debug.task.velocity,(0.,0.,0.))
                if scenario=='complete':
                    pump_until(lambda: debug.task.finished,timeout=25.)
                    self.assertEqual(debug.task.state,'DONE',debug.task.reason)
                    self.assertTrue(debug.task.returned)
                    self.assertIsNotNone(debug.task.reference)
                    self.assertEqual([x[2] for x in debug.task.confirmed],[2,1,3])
                    self.assertEqual([data[0] for _,mid,data in board.frames if mid==ARM_CMD],[3,3,3,3,2])
                    for i,phase in enumerate(phases[:-1]):
                        if phase[0]=='PICK':
                            self.assertGreaterEqual(phases[i+1][2]-phase[2],1.)
                            self.assertEqual(phases[i+1][0],'RESTORE_WORK')
                    states=[p[0] for p in phases]
                    self.assertEqual(states.count('PICK'),3)
                    self.assertIn('RETURN_REFERENCE',states)
                    self.assertNotIn('CAPTURE_REFERENCE',states)
                    pump_until(lambda: request('/api/state')['state']=='DONE')
                    snapshot=request('/api/state')
                    self.assertFalse(snapshot['base_locked'])
                    self.assertEqual(snapshot['reference'],list(debug.task.reference))
                    self.assertIn('arm_velocity',snapshot)
                    self.assertEqual(len(snapshot['confirmed']),3)
                else:
                    scene.hidden=True
                    pump_until(lambda: debug.task.state=='WAIT_REQUIRED')
                    start=time.monotonic()
                    pump_until(lambda: time.monotonic()-start>1.)
                    self.assertEqual(debug.task.state,'WAIT_REQUIRED')
                    self.assertEqual([data[0] for _,mid,data in board.frames if mid==ARM_CMD],[3])
                    scene.hidden=False
                    pump_until(lambda: debug.task.servo_enabled)
                    command('stop')
                    pump_until(lambda: debug._arm_idle() and not board.jog)
                    self.assertEqual(debug.task.state,'STOPPED')
                    self.assertEqual(board.command[:3],(0.,0.,0.))
                    pump_until(lambda: debug._can_start(scene.latest()))
                    command('start',dict(area='raw',order='213'))
                    pump_until(lambda: debug.task.state=='ERROR',timeout=3.,heartbeat=False)
                    self.assertIn('页面失联',debug.task.reason)
                    pump_until(lambda: debug._arm_idle() and not board.jog,heartbeat=False)
                    self.assertEqual(board.command[:3],(0.,0.,0.))
                jogs=[JOG_STRUCT.unpack(data) for _,mid,data in board.frames if mid==ARM_JOG]
                self.assertTrue(any(j[3] == 1 and (abs(j[0])>0 or abs(j[1])>0) for j in jogs))
                self.assertEqual(jogs[-1][3],0)
                self.assertFalse(any(mid==ARM_CMD and data[0]==4 for _,mid,data in board.frames))
                base_commands=[CHASSIS_STRUCT.unpack(data) for _,mid,data in board.frames if mid==CHASSIS_CMD]
                self.assertTrue(any(cmd[0]>0 for cmd in base_commands))
                if scenario=='complete':
                    self.assertTrue(any(cmd[0]<0 for cmd in base_commands))
                    self.assertAlmostEqual(board.x,debug.task.reference[0],delta=.001)
                self.assertEqual(board.command[:3],(0.,0.,0.))
                self.assertIsNone(board.error)
            finally:
                for node in reversed(nodes):
                    if hasattr(node,'close'):
                        node.close()
                    if executor:
                        executor.remove_node(node)
                    node.destroy_node()
                if executor:
                    executor.shutdown()
                board.close()
                rclpy.shutdown()
                sys.setswitchinterval(switch_interval)


if __name__=='__main__':
    unittest.main()
