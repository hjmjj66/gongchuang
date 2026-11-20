"""HTTP/ROS calibration with synthetic frames and no vehicle, bridge or camera."""

from agv_perception.work_config import read_work_config

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


class CalibrationVision:
    ready, error, kind, frozen = True, '', 'ring', False

    def __init__(self):
        self.sequence, self.point, self.obs = 0, (412.25,333.5), None

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        if kind != self.kind:
            self.kind, self.obs = kind, None

    def latest(self):
        if self.frozen:
            return self.obs
        self.sequence += 1
        self.obs=Observation(self.kind,self.sequence,time.monotonic(),(1280,720),
            {2:self.point} if self.kind=='ring' else {},((5,self.point),) if self.kind=='material' else ())
        return self.obs

    def packet(self):
        return (self.obs,b'calibration-frame') if self.obs else None

    def preview(self):
        return b'calibration-frame'

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST')=='1','opt in to local HTTP/ROS test')
class CalibrationRosTests(unittest.TestCase):
    def test_capture_without_pc_feedback_sync_three_points_and_reject_stale_frames(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from coarse_debug.node import DebugNode

        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','185')))
        debug=executor=None
        root=Path(__file__).resolve().parents[3]
        try:
            with tempfile.TemporaryDirectory(prefix='calibration_http_') as folder:
                folder=Path(folder)
                config=read_work_config((root/'tools/coarse_debug/config/debug.yaml'))
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                for area in ('raw','coarse','storage'):
                    path=folder/f'{area}.yaml'
                    formal=read_work_config(root/f'ros2_ws/src/agv_perception/config/{area}.yaml')
                    formal.pop('motion_config', None)
                    path.write_text(yaml.safe_dump(formal,allow_unicode=True))
                    config['debug'][f'calibration_{area}_config']=str(path)
                path=folder/'debug.yaml'
                path.write_text(yaml.safe_dump(config,allow_unicode=True))
                debug=DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=CalibrationVision())
                executor=SingleThreadedExecutor()
                executor.add_node(debug)
                base=f'http://127.0.0.1:{debug.web.http.server_port}'
                client='calibration-browser'

                def request(route,payload=None):
                    req=Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=2.) as response:
                        body=response.read()
                        return (body,json.loads(response.headers['X-Preview-Observation'])) if route=='/preview.jpg' else json.loads(body)

                def pump(predicate,timeout=3.):
                    until=time.monotonic()+timeout
                    while time.monotonic()<until:
                        executor.spin_once(timeout_sec=.005)
                        if predicate():
                            return
                    self.fail(f'timed out, mode={debug.mode}, reason={debug.task and debug.task.reason}')

                def command(action,settings=None,who=client):
                    operation=uuid.uuid4().hex
                    request('/api/command',dict(client=who,operation_id=operation,action=action,settings=settings or {}))
                    pump(lambda:not debug.web.operations[(who,operation)][1]['pending'])
                    return debug.web.operations[(who,operation)][1]

                previous=None
                for category,target in (('raw',5),('ring',2),('stack',5)):
                    result=command('select',dict(mode='calibration',calibration_type=category))
                    self.assertTrue(result['ok'],result)
                    self.assertFalse(debug._healthy())  # No PC/odometry/arm feedback exists in this test.
                    self.assertIsNone(debug.task)
                    pump(lambda:debug.vision.obs and debug.vision.obs.captured_at>debug.calibration_context[2])
                    body,frame=request('/preview.jpg')
                    self.assertEqual(body,b'calibration-frame')
                    self.assertEqual(frame['category'],category)
                    point=list(debug.vision.point)
                    debug.vision.point=(point[0]+10.,point[1]+10.)
                    pump(lambda:debug.vision.obs.sequence>frame['sequence'])
                    result=command('calibration_capture',dict(frame_token=frame['token'],target=target))
                    self.assertTrue(result['ok'],result)
                    sample=debug.calibration_sample
                    self.assertEqual(sample['point'],point)
                    self.assertEqual(sample['sequence'],frame['sequence'])
                    self.assertFalse(command('calibration_save',dict(sample_id=sample['id']),who='another-browser')['ok'])
                    self.assertTrue(command('calibration_save',dict(sample_id=sample['id']))['ok'])
                    self.assertTrue(debug.calibration_sample['saved'])
                    from coarse_debug.config import area_fields, get_value
                    selection = debug.calibration_context
                    area = debug.area
                    values = {item['path']: get_value(debug.config, item['path'], area) for item in area_fields(area)}
                    gain = 'gantry_servo.yaw_pid.kp' if area == 'raw' else 'gantry_servo.v_pid.kp'
                    tolerance = 'grasp_gate.tolerance_px' if area == 'raw' else 'gantry_servo.tolerance_px'
                    values.update({gain: .12, 'gantry_servo.max_fb_rpm': 4., tolerance: 9.})
                    result = command('save', values)
                    self.assertTrue(result['ok'], result)
                    self.assertEqual(debug.mode, 'calibration')
                    self.assertEqual(debug.calibration_context, selection)
                    self.assertIs(debug.calibration_sample, sample)
                    self.assertEqual(get_value(debug.config, gain, area), .12)
                    formal = read_work_config(folder/f'{area}.yaml')
                    self.assertEqual(formal[area]['gantry_servo']['max_fb_rpm'], 4.)
                    self.assertEqual(formal[area][tolerance.split('.')[0]]['tolerance_px'], 9.)
                    if area != 'raw':
                        self.assertEqual(debug.config['coarse']['gantry_servo'], debug.config['storage']['gantry_servo'])
                    destinations={'raw':(('raw','target_uv_px'),),'ring':(('coarse','target_uv_px'),('storage','target_uv_px')),
                                  'stack':(('storage','stack_target_uv_px'),)}[category]
                    for area,key in destinations:
                        formal=yaml.safe_load((folder/f'{area}.yaml').read_text())
                        web=yaml.safe_load(path.read_text())
                        self.assertEqual(formal[area]['vision'][key],point)
                        self.assertEqual(web[area]['vision'][key],point)
                    if previous:
                        self.assertFalse(command('calibration_capture',dict(frame_token=previous,target=target))['ok'])
                    previous=frame['token']
                    self.assertFalse(command('start',dict(mode='calibration'))['ok'])
                    self.assertFalse(debug._servo.stream_id)
                    self.assertIsNone(debug._arm_future)

                pump(lambda:debug.web.snapshot()['calibration_sample']['saved'])
                snapshot=request('/api/state')
                self.assertTrue(snapshot['fixed_parameters'])
                self.assertEqual(snapshot['calibration_points']['ring'],[422.25,343.5])
                self.assertEqual(snapshot['calibration_points']['stack'],[432.25,353.5])
                self.assertTrue(command('select',dict(mode='task', area='raw'))['ok'])
                from coarse_debug.config import area_fields, get_value
                values = {item['path']: get_value(debug.config, item['path'], 'raw') for item in area_fields('raw')}
                values['gantry_servo.max_fb_rpm'] = 4.
                self.assertTrue(command('save', values)['ok'])
                formal = yaml.safe_load((folder/'raw.yaml').read_text())
                self.assertEqual(formal['raw'], debug.config['raw'])
                self.assertTrue(formal['raw']['motion_prediction']['enabled'])
                self.assertEqual(formal['raw']['gantry_servo']['max_fb_rpm'], 4.)
                self.assertTrue(command('select',dict(mode='tracking', area='raw'))['ok'])
                values['gantry_servo.max_fb_rpm'] = 5.
                self.assertTrue(command('save', values)['ok'])
                self.assertEqual(debug.mode, 'tracking')
                self.assertEqual(read_work_config(folder/'raw.yaml')['raw']['gantry_servo']['max_fb_rpm'], 5.)
                self.assertTrue(command('select',dict(mode='calibration',calibration_type='shared_pose'))['ok'])
                values = {item['path']: get_value(debug.config, item['path'], 'coarse') for item in area_fields('coarse')}
                values['gantry_motion.max_fb_rpm'] = 6.
                self.assertTrue(command('save', values)['ok'])
                self.assertEqual(debug.mode, 'calibration')
                self.assertEqual(debug.calibration_context[0], 'shared_pose')
                self.assertEqual(read_work_config(folder/'storage.yaml')['storage']['gantry_motion']['max_fb_rpm'], 6.)
                self.assertTrue(command('select',dict(mode='calibration',calibration_type='stack'))['ok'])
                pump(lambda:debug.vision.obs and debug.vision.obs.captured_at>debug.calibration_context[2])
                debug.vision.frozen=True
                _,frame=request('/preview.jpg')
                pump(lambda:time.monotonic()-debug.vision.obs.captured_at>.6)
                result=command('calibration_capture',dict(frame_token=frame['token'],target=5))
                self.assertFalse(result['ok'])
                self.assertIn('过期',result['error'])
        finally:
            if debug:
                debug.close()
                if executor:
                    executor.remove_node(debug)
                debug.destroy_node()
            if executor:
                executor.shutdown()
            rclpy.shutdown()
