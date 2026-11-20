import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen

import cv2
import numpy as np

from agv_perception.material_config import parse_material_config
from agv_perception.material_detector import MaterialDetector
from server import FrameRate,ImageSource,MaterialServer,MaterialSession


class MutableSource:
    device = 'test camera input'
    def __init__(self,frame):
        self.frame=(1,frame,12.0,time.monotonic())
    def latest(self): return self.frame
    def start(self): pass
    def close(self): self.frame=None


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)/'image.png'
        frame=np.full((200,300,3),255,np.uint8)
        cv2.circle(frame,(100,100),30,(0,0,200),-1)
        cv2.imwrite(str(self.path),frame)
        self.frame=frame
        self.p=parse_material_config({'material_backend':1,'max_frame_age_s':.15})

    def tearDown(self):
        self.tmp.cleanup()

    def wait_packet(self,session):
        deadline=time.monotonic()+3
        while time.monotonic() < deadline:
            packet=session.snapshot()
            if packet['state']=='READY': return packet
            time.sleep(.01)
        self.fail(f'no READY packet: {session.snapshot()["message"]}')

    def test_static_image_detects_once_and_json_matches_image_sequence(self):
        detector=MaterialDetector(self.p)
        source=ImageSource(self.path)
        session=MaterialSession(source,detector,self.p,image_mode=True)
        session.start()
        try:
            packet=self.wait_packet(session)
            time.sleep(.2)
            again=session.snapshot()
            self.assertEqual(packet['result'],again['result'])
            self.assertEqual(packet['display_frame_seq'],packet['result']['frame_seq'])
            self.assertEqual(packet['result']['observations'][0]['color_id'],1)
            self.assertIsNone(packet['camera_fps'])
            self.assertIsNone(packet['processing_fps'])
            self.assertTrue(packet['image_base64'])
            json.dumps(packet,allow_nan=False)
        finally:
            session.close()
        self.assertFalse(session.thread.is_alive())

    def test_stale_and_disconnected_camera_clear_all_centers(self):
        source=MutableSource(self.frame)
        session=MaterialSession(source,MaterialDetector(self.p),self.p)
        session.start()
        try:
            self.wait_packet(session)
            source.frame=(2,self.frame,12.1,time.monotonic()-2)
            packet=session.snapshot()
            self.assertEqual(packet['state'],'STALE_FRAME')
            self.assertIsNone(packet['result'])
            self.assertIsNone(packet['image_base64'])
            source.frame=None
            packet=session.snapshot()
            self.assertEqual(packet['state'],'NO_FRAME')
            self.assertIsNone(packet['result'])
        finally:
            session.close()

    def test_stale_result_and_inference_error_clear_centers_but_allow_raw_image(self):
        source=MutableSource(self.frame)
        session=MaterialSession(source,MaterialDetector(self.p),self.p)
        session.packet=({'frame_seq':1,'observations':[{'center_px':[1,2]}]},b'old jpeg',time.monotonic()-1)
        packet=session.snapshot()
        self.assertEqual(packet['state'],'STALE_RESULT')
        self.assertIsNone(packet['result'])
        self.assertTrue(packet['image_base64'])
        session._record_error(RuntimeError('test error'))
        packet=session.snapshot()
        self.assertEqual(packet['state'],'INFERENCE_ERROR')
        self.assertIsNone(packet['result'])

    def test_http_serves_dynamic_results_and_rejects_unknown_paths(self):
        session=MaterialSession(ImageSource(self.path),MaterialDetector(self.p),self.p,image_mode=True)
        session.start()
        server=None
        try:
            self.wait_packet(session)
            server=MaterialServer(('127.0.0.1',0),session)
            thread=threading.Thread(target=server.serve_forever,daemon=True)
            thread.start()
            base=f'http://127.0.0.1:{server.server_port}'
            with urlopen(base+'/api/frame') as response:
                packet=json.load(response)
                self.assertEqual(response.headers['Cache-Control'],'no-store')
                self.assertEqual(packet['display_frame_seq'],packet['result']['frame_seq'])
            for path in ('/','/app.js','/style.css'):
                with urlopen(base+path) as response:
                    self.assertEqual(response.status,200)
        finally:
            if server:
                server.shutdown()
                server.server_close()
                thread.join(2)
            session.close()

    def test_capture_rate_counts_skipped_sequences(self):
        rate=FrameRate()
        rate.observe(10,1)
        rate.observe(40,2)
        self.assertEqual(rate.value(2),30)
        rate.observe(40,2.1)
        self.assertEqual(rate.value(2.1),30)
        self.assertEqual(rate.value(4),0)

    def test_failed_switch_retains_current_detector_and_reprocesses_image(self):
        p = parse_material_config({**self.p,'jhb.model_path':'/missing/jhb.onnx'})
        detector = MaterialDetector(p)
        session = MaterialSession(ImageSource(self.path),detector,p,image_mode=True)
        session.start()
        try:
            self.wait_packet(session)
            session.request_mode(True)
            immediate = session.snapshot()
            self.assertIsNone(immediate['result'])
            packet = self.wait_packet(session)
            self.assertFalse(packet['use_jhb_model'])
            self.assertIn('切换失败',packet['switch_error'])
            self.assertIs(session.detector,detector)
            self.assertEqual(packet['result']['backend'],'hsv')
            self.assertEqual(packet['result']['mode_revision'],1)
        finally:
            session.close()

    def test_switch_during_inference_discards_old_result_and_rejects_duplicate(self):
        entered, release = threading.Event(),threading.Event()
        class DelayedDetector(MaterialDetector):
            def detect(self,*args,**kwargs):
                result = super().detect(*args,**kwargs)
                entered.set()
                release.wait(3)
                return result
        p = parse_material_config({**self.p,'jhb.model_path':'/missing/jhb.onnx'})
        session = MaterialSession(ImageSource(self.path),DelayedDetector(p),p,image_mode=True)
        session.start()
        try:
            self.assertTrue(entered.wait(2))
            session.request_mode(True)
            with self.assertRaises(RuntimeError):
                session.request_mode(False)
            self.assertIsNone(session.snapshot()['result'])
            release.set()
            packet = self.wait_packet(session)
            self.assertEqual(packet['mode_revision'],1)
            self.assertEqual(packet['result']['mode_revision'],1)
            self.assertIn('切换失败',packet['switch_error'])
        finally:
            release.set()
            session.close()

    def test_slow_result_is_never_saved_as_a_valid_current_target(self):
        class DelayedDetector(MaterialDetector):
            def detect(self, *args, **kwargs):
                result=super().detect(*args,**kwargs)
                time.sleep(.2)
                return result
        p=parse_material_config({**self.p,'output_dir':self.tmp.name,'max_frame_age_s':.1})
        source=MutableSource(self.frame)
        session=MaterialSession(source,DelayedDetector(p),p)
        session.start()
        try:
            path=Path(self.tmp.name)/'results.jsonl'
            deadline=time.monotonic()+2
            while time.monotonic() < deadline and not path.exists():
                time.sleep(.01)
            self.assertTrue(path.exists())
            saved=json.loads(path.read_text().strip())
            self.assertEqual(saved['status'],'STALE_RESULT')
            self.assertEqual(saved['observations'],[])
            self.assertIsNone(session.snapshot()['result'])
        finally:
            session.close()


if __name__ == '__main__':
    unittest.main()
