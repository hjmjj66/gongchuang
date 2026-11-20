"""Real JHB image inference through the existing preview HTTP API."""

import json
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2

from agv_perception.material_config import JHB_DEPLOY, parse_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_model import load_material_model
from server import ImageSource, MaterialServer, MaterialSession

ROOT = Path(JHB_DEPLOY)
ONNX = ROOT/'detector/material_cap_v218_384x640_9a6d18d0cc69.onnx'


@unittest.skipUnless(ONNX.is_file(),'JHB package not installed')
class JhbHttpTests(unittest.TestCase):
    def test_combine_http_roundtrip_skips_lab_and_keeps_unknown_centers(self):
        cv2.setNumThreads(2)
        p = parse_material_config({'material_backend':1, 'jhb.model_path':str(ONNX),
                                   'jhb.color_path':'/missing/not-used-in-combine.json'})
        session = MaterialSession(ImageSource(ROOT/'detector/selftest.jpg'),
                                  MaterialDetector(p), p, image_mode=True)
        server = MaterialServer(('127.0.0.1',0),session)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        session.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            # Only the old X5 predictor is unavailable; JHB inference remains real.
            with patch('agv_perception.material_combine.load_material_model',
                       return_value=SimpleNamespace(infer=lambda image: [])):
                for revision, combine in enumerate((True, False), 1):
                    request = Request(base+'/api/mode',
                                      json.dumps(dict(use_jhb_model=False,test_combine=combine)).encode(),
                                      {'Content-Type':'application/json'}, method='POST')
                    with urlopen(request, timeout=3) as response:
                        self.assertEqual(response.status,202)
                    deadline = time.monotonic()+15
                    while time.monotonic() < deadline:
                        with urlopen(base+'/api/frame',timeout=3) as response:
                            packet = json.load(response)
                        if packet['state']=='READY' and packet['mode_revision']==revision:
                            break
                        time.sleep(.01)
                    self.assertEqual(packet['state'],'READY',packet['message'])
                    self.assertIsNone(packet['switch_error'])
                    self.assertEqual(packet['test_combine'],combine)
                    self.assertEqual(packet['backend'],'test_combine' if combine else 'hsv')
                    if combine:
                        items = packet['result']['observations']
                        self.assertEqual(len(items),3)
                        self.assertTrue(all(o['color_id']==0 and o['center_px'] is not None for o in items))
                        self.assertIn('color_yolo',packet['result']['stages_ms'])
                        self.assertNotIn('color',packet['result']['stages_ms'])
                for invalid in (dict(use_jhb_model=True,test_combine=True),
                                dict(use_jhb_model=False,test_combine='true')):
                    request = Request(base+'/api/mode',json.dumps(invalid).encode(),
                                      {'Content-Type':'application/json'},method='POST')
                    with self.assertRaises(HTTPError) as ctx:
                        urlopen(request,timeout=3)
                    self.assertEqual(ctx.exception.code,400)
        finally:
            server.shutdown()
            server.server_close()
            session.close()
            thread.join(2)

    def test_jhb_metadata_and_real_image_result(self):
        cv2.setNumThreads(2)
        p = parse_material_config({'use_jhb_model':True,'material_backend':1,
                                   'material_model':2,'jhb.model_path':str(ONNX)})
        source = ImageSource(ROOT/'detector/selftest.jpg')
        session = MaterialSession(source,MaterialDetector(p,load_material_model(p)),p,image_mode=True)
        server = MaterialServer(('127.0.0.1',0),session)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        session.start()
        try:
            url = f'http://127.0.0.1:{server.server_port}/api/frame'
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                with urlopen(url,timeout=3) as response:
                    packet = json.load(response)
                if packet['state'] in ('READY','INFERENCE_ERROR'):
                    break
                time.sleep(.02)
            self.assertEqual(packet['state'],'READY',packet['message'])
            self.assertEqual(packet['backend'],'jhb')
            self.assertEqual(packet['class_labels'],['material','cap_top'])
            self.assertEqual(packet['model_path'],str(ONNX))
            self.assertEqual(packet['display_frame_seq'],packet['result']['frame_seq'])
            self.assertEqual(len(packet['result']['observations']),3)
            self.assertTrue(packet['image_base64'])
            self.assertEqual({o['color_id'] for o in packet['result']['observations'] if o['valid']},{3,4})
            for use_jhb,expected_backend in ((False,'hsv'),(True,'jhb'),(False,'hsv')):
                request = Request(url.replace('/frame','/mode'),
                                  json.dumps({'use_jhb_model':use_jhb}).encode(),
                                  {'Content-Type':'application/json'},method='POST')
                with urlopen(request,timeout=3) as response:
                    self.assertEqual(response.status,202)
                revision = packet['mode_revision']+1
                deadline = time.monotonic()+15
                while time.monotonic() < deadline:
                    with urlopen(url,timeout=3) as response:
                        packet = json.load(response)
                    if packet['switching']:
                        self.assertIsNone(packet['result'])
                    if packet['state']=='READY' and packet['mode_revision']==revision:
                        break
                    time.sleep(.01)
                self.assertEqual(packet['state'],'READY',packet['message'])
                self.assertIsNone(packet['switch_error'])
                self.assertEqual(packet['backend'],expected_backend)
                self.assertEqual(packet['result']['backend'],expected_backend)
                self.assertEqual(packet['result']['mode_revision'],revision)
                self.assertEqual(packet['display_frame_seq'],1)  # Same static image reprocessed.
                if use_jhb:
                    self.assertEqual(len(packet['result']['observations']),3)
            for invalid in ({'use_jhb_model':'true'},{'mode':'jhb'},[]):
                request = Request(url.replace('/frame','/mode'),json.dumps(invalid).encode(),
                                  {'Content-Type':'application/json'},method='POST')
                with self.assertRaises(HTTPError) as ctx:
                    urlopen(request,timeout=3)
                self.assertEqual(ctx.exception.code,400)
        finally:
            server.shutdown()
            server.server_close()
            session.close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()
