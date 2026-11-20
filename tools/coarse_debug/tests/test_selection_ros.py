"""Mode selection without arm/PC feedback; no vehicle, bridge or camera."""

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

from test_calibration_ros import CalibrationVision


class SwitchableVision(CalibrationVision):
    use_jhb_model, test_combine, switching = False, False, False

    def material_model_state(self):
        return dict(available=True, use_jhb_model=self.use_jhb_model,
                    test_combine=self.test_combine,
                    switching=self.switching, error='')

    def request_material_model(self, use_jhb_model, *, test_combine=False):
        self.requested_mode=use_jhb_model
        self.requested_combine=test_combine
        self.switching=True


@unittest.skipUnless(os.environ.get('AGV_COARSE_DEBUG_ROS_TEST')=='1','opt in to local HTTP/ROS test')
class SelectionRosTests(unittest.TestCase):
    def test_select_without_feedback_but_keep_start_and_active_guards(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from coarse_debug.node import DebugNode

        rclpy.init(domain_id=int(os.environ.get('ROS_DOMAIN_ID','185')))
        debug=executor=None
        root=Path(__file__).resolve().parents[3]
        try:
            with tempfile.TemporaryDirectory(prefix='selection_http_') as folder:
                folder=Path(folder)
                config=read_work_config((root/'tools/coarse_debug/config/debug.yaml'))
                config.pop('motion_config', None)  # Isolated expanded simulation settings.
                for area in ('raw','coarse','storage'):
                    path=folder/f'{area}.yaml'
                    path.write_text((root/f'ros2_ws/src/agv_perception/config/{area}.yaml').read_text())
                    config['debug'][f'calibration_{area}_config']=str(path)
                path=folder/'debug.yaml'
                path.write_text(yaml.safe_dump(config,allow_unicode=True))
                debug=DebugNode(SimpleNamespace(config=str(path),host='127.0.0.1',port=0),vision=SwitchableVision())
                executor=SingleThreadedExecutor()
                executor.add_node(debug)
                base=f'http://127.0.0.1:{debug.web.http.server_port}'
                client='selection-browser'

                def request(route,payload=None):
                    req=Request(base+route,data=None if payload is None else json.dumps(payload).encode(),
                                headers={'Content-Type':'application/json'})
                    with urlopen(req,timeout=2.) as response:
                        return json.loads(response.read())

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

                # Reproduce startup/stop without an arm/PC connection: selecting
                # any preview mode must work, but starting motion must still fail.
                debug.task=SimpleNamespace(finished=False)
                try:
                    with self.assertRaisesRegex(ValueError,'任务正在运行'):
                        debug._command('select',dict(mode='tracking'),client)
                    with self.assertRaisesRegex(ValueError,'任务正在运行'):
                        debug._command('material_model',dict(use_jhb_model=True),client)
                finally:
                    debug.task=None
                debug._arm_future=object()
                try:
                    with self.assertRaisesRegex(ValueError,'指令仍在等待响应'):
                        debug._command('select',dict(mode='tracking'),client)
                    with self.assertRaisesRegex(ValueError,'指令仍在等待响应'):
                        debug._command('material_model',dict(use_jhb_model=True),client)
                finally:
                    debug._arm_future=None
                for mode,area,visit in (('tracking','coarse',1),('task','raw',1),
                                        ('tracking','storage',2),('task','coarse',1)):
                    result=command('select',dict(mode=mode,area=area,visit=visit))
                    self.assertTrue(result['ok'],result)
                    self.assertEqual((debug.mode,debug.area,debug.visit),(mode,area,visit))
                    self.assertFalse(debug._arm_idle())
                    self.assertFalse(debug._can_start(debug.vision.latest()))
                    settings=dict(mode=mode,area=area,visit=visit,target=5 if area=='storage' else 2,
                                  order='213')
                    result=command('start',settings)
                    self.assertFalse(result['ok'])
                    self.assertIn('尚未就绪',result['error'])
                    self.assertIsNone(debug.task)
                    self.assertIsNone(debug._arm_future)
                    self.assertFalse(debug._servo.stream_id)
                    self.assertTrue(command('stop')['ok'])

                self.assertTrue(command('select',dict(mode='calibration',calibration_type='raw'))['ok'])
                old_context=debug.calibration_context
                old_frames=debug.calibration_frames
                metadata=old_frames.register(debug.vision.latest(),*old_context[:2])
                debug.calibration_sample={'id':'old-model-sample'}
                debug.reference_ready=('coarse',None)
                result=command('material_model',dict(use_jhb_model=True))
                self.assertTrue(result['ok'],result)
                self.assertTrue(debug.vision.requested_mode)
                self.assertNotEqual(old_context[1],debug.calibration_context[1])
                self.assertIsNone(debug.calibration_sample)
                self.assertIsNone(debug.reference_ready)
                self.assertFalse(debug.calibration_frames.frames)
                result=command('calibration_capture',dict(frame_token=metadata['token'],target=5))
                self.assertFalse(result['ok'])
                self.assertIn('画面已切换',result['error'])
                pump(lambda:request('/api/state').get('material_model',{}).get('switching'))
                self.assertTrue(request('/api/state')['material_model']['switching'])
                # Loading must also block motion paths which do not use _can_start.
                result=command('preset_track',dict(target=3))
                self.assertFalse(result['ok'])
                self.assertIn('正在切换',result['error'])
                debug.vision.switching=False
                debug.vision.use_jhb_model=True
                pump(lambda:request('/api/state').get('material_model',{}).get('use_jhb_model'))
                result=command('material_model',dict(use_jhb_model=False,test_combine=True))
                self.assertTrue(result['ok'],result)
                self.assertFalse(debug.vision.requested_mode)
                self.assertTrue(debug.vision.requested_combine)

        finally:
            if debug:
                debug.close()
                if executor:
                    executor.remove_node(debug)
                debug.destroy_node()
            if executor:
                executor.shutdown()
            rclpy.shutdown()
