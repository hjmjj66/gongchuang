"""Real X5 model and preview pipeline with a still-photo camera boundary, no serial."""

import logging
import os
from pathlib import Path
import time
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('AGV_TEST_X5_COARSE_DEBUG') == '1', 'opt in to real X5 BPU inference')
class VisionTests(unittest.TestCase):
    def test_material_process_preserves_real_model_centers_and_classification(self):
        import cv2
        from agv_perception.material_config import read_material_config
        from agv_perception.material_detector import MaterialDetector
        from agv_perception.material_model import load_material_model
        from agv_perception.material_worker import MaterialWorker

        config=read_material_config(Path(__file__).parents[3]/'ros2_ws/src/agv_perception/config/material.yaml')
        for jhb,image in ((False,'/root/material_deploy_v1/check_black_blues.jpg'),
                          (True,'/root/去年开源/gc_material_model_v218_20261116/detector/selftest.jpg')):
            with self.subTest(jhb=jhb):
                p={**config,'use_jhb_model':jhb}
                pixels=cv2.imread(image)
                direct=MaterialDetector(p,load_material_model(p))
                worker=MaterialWorker(p)
                try:
                    for target in (0,3):
                        expected=direct.detect(pixels,7,1.5,geometry_target=target)
                        result=worker.detect(pixels,7,1.5,geometry_target=target)
                        self.assertIsNone(result.error)
                        self.assertTrue(result.observations)
                        self.assertEqual(result.observations,expected.observations)
                        self.assertEqual((result.status,result.image_size),(expected.status,expected.image_size))
                finally:
                    worker.close()

    def test_real_ring_material_switches_share_camera_and_drop_old_packets(self):
        import cv2
        import numpy as np
        from coarse_debug.config import read_config
        from coarse_debug.vision import RingVision

        ring=cv2.imread('/root/ring_deploy_v1_20261103/images/check_254.jpg')
        material=cv2.imread('/root/material_deploy_v1/check_black_blues.jpg')
        self.assertIsNotNone(ring)
        self.assertIsNotNone(material)
        cameras=[]

        class StillCamera:
            pixels=ring
            def __init__(self,*args,**kwargs):
                self.sequence=0
                cameras.append(self)
            def start(self):
                pass
            def latest(self):
                self.sequence+=1
                at=time.monotonic()
                return self.sequence,self.pixels,at,at
            def close(self):
                pass

        config=read_config(Path(__file__).parents[1]/'config/debug.yaml')
        with patch('agv_perception.work_vision.CameraStream',StillCamera):
            vision=RingVision(config['coarse']['vision']['ring_config'],960,logging.getLogger('test_switch'),
                              config['storage']['vision']['material_config'])
            try:
                def wait_kind(kind):
                    until=time.monotonic()+20.
                    while time.monotonic()<until and not vision.error:
                        obs=vision.latest()
                        packet=vision.packet()
                        if obs is not None and obs.kind==kind and packet is not None and packet[0].kind==kind:
                            return obs
                        time.sleep(.02)
                    self.fail(f'no {kind} observation: {vision.error}')
                self.assertEqual(set(wait_kind('ring').rings),{1,2,3})
                StillCamera.pixels=material
                vision.request('material')
                self.assertIsNone(vision.latest())
                self.assertIsNone(vision.preview())
                obs=wait_kind('material')
                self.assertEqual({c for c,_ in obs.materials},{3,5,6})
                jpeg=cv2.imdecode(np.frombuffer(vision.preview(),dtype=np.uint8),cv2.IMREAD_COLOR)
                self.assertEqual(jpeg.shape[1],960)
                StillCamera.pixels=ring
                vision.request('ring')
                self.assertEqual(set(wait_kind('ring').rings),{1,2,3})
                self.assertEqual(len(cameras),1)
            finally:
                vision.close()

    def test_real_ring_model_worker_and_matching_jpeg(self):
        import cv2
        import numpy as np
        from coarse_debug.config import read_config
        from coarse_debug.vision import RingVision

        image=Path('/root/ring_deploy_v1_20261103/images/check_254.jpg')
        pixels=cv2.imread(str(image))
        self.assertIsNotNone(pixels)
        class StillCamera:
            def __init__(self,*args,**kwargs):
                self.packet=None
            def start(self):
                at=time.monotonic()
                self.packet=(1,pixels,at,at)
            def latest(self):
                return self.packet
            def close(self):
                pass
        config=read_config(Path(__file__).parents[1]/'config/debug.yaml')
        with patch('agv_perception.work_vision.CameraStream',StillCamera):
            vision=RingVision(config['coarse']['vision']['ring_config'],960,logging.getLogger('test_vision'))
            try:
                until=time.monotonic()+20.
                while vision.packet() is None and not vision.error and time.monotonic()<until:
                    time.sleep(.02)
                self.assertFalse(vision.error,vision.error)
                obs=vision.latest()
                self.assertIsNotNone(obs)
                self.assertEqual(set(obs.rings),{1,2,3})
                self.assertEqual(obs.image_size,(pixels.shape[1],pixels.shape[0]))
                jpeg=cv2.imdecode(np.frombuffer(vision.preview(),dtype=np.uint8),cv2.IMREAD_COLOR)
                self.assertEqual(jpeg.shape[1],960)
                self.assertEqual(obs.sequence,1)
            finally:
                vision.close()


if __name__=='__main__':
    unittest.main()
