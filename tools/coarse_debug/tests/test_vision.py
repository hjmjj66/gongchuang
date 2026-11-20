"""Real geometry and JPEG pipeline with explicit camera/model test inputs."""

import logging
from pathlib import Path
import threading
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from agv_perception.ring_model import ModelDetection
from coarse_debug.vision import RingVision


class CameraFixture:
    def __init__(self):
        self.frame = None
        self.closed = False

    def start(self):
        pixels = np.full((320, 480, 3), 240, np.uint8)
        cv2.circle(pixels, (230, 150), 66, (0, 0, 0), 3, cv2.LINE_AA)
        self.pixels = pixels
        self.advance(1)

    def advance(self, sequence):
        now = time.monotonic()
        self.frame = sequence, self.pixels, now, now

    def latest(self):
        return self.frame

    def close(self):
        self.closed = True


class ModelFixture:
    def infer(self, pixels):
        return [ModelDetection('1', .9, (20, 20, 100, 100)),
                ModelDetection('2', .9, (150, 70, 310, 230)),
                ModelDetection('3', .9, (360, 20, 440, 100))]


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.entered, self.release = threading.Event(), threading.Event()
        entered, release = self.entered, self.release

        class SlowPreview(RingVision):
            def _encode_preview(self, *args):
                entered.set()
                if not release.wait(timeout=5.):
                    raise RuntimeError('test did not release preview encoder')
                return super()._encode_preview(*args)

        self.camera = CameraFixture()
        camera_patch = patch('agv_perception.work_vision.CameraStream', return_value=self.camera)
        model_patch = patch('agv_perception.work_vision.load_ring_model', return_value=ModelFixture())
        camera_patch.start()
        model_patch.start()
        self.addCleanup(camera_patch.stop)
        self.addCleanup(model_patch.stop)
        config = Path(__file__).resolve().parents[3]/'ros2_ws/src/agv_perception/config/ring.yaml'
        folder = tempfile.TemporaryDirectory(prefix='raw_preview_')
        self.addCleanup(folder.cleanup)
        material_path = Path(folder.name)/'material.yaml'
        material = yaml.safe_load((config.parent/'material.yaml').read_text())
        material['material_debug_node']['ros__parameters'].update(material_backend=1, use_jhb_model=False)
        material_path.write_text(yaml.safe_dump(material))
        self.vision = SlowPreview(config, 240, logging.getLogger('test_preview'), material_path)
        self.addCleanup(self.close_vision)

    def close_vision(self):
        self.release.set()
        self.vision.close()
        self.assertTrue(self.camera.closed)
        self.assertFalse(self.vision._thread.is_alive())

    def wait_for(self, predicate):
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            self.assertFalse(self.vision.error, self.vision.error)
            if predicate():
                return
            time.sleep(.005)
        self.fail('vision worker did not publish the expected result')

    def test_blocked_preview_does_not_delay_control_and_packet_keeps_its_own_frame(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        for sequence in (2, 3):
            self.camera.advance(sequence)
            self.wait_for(lambda: self.vision.latest().sequence == sequence)
            obs = self.vision.latest()
            self.assertAlmostEqual(obs.rings[2][0], 230, delta=3)
            self.assertAlmostEqual(obs.rings[2][1], 150, delta=3)
            self.assertIsNone(self.vision.packet())
            self.assertIsNone(self.vision.preview())
        self.release.set()
        self.wait_for(lambda: self.vision.packet() is not None)
        obs, jpeg = self.vision.packet()
        self.assertEqual(obs.sequence, 1)
        self.assertEqual(self.vision.latest().sequence, 3)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (160, 240, 3))
        self.camera.advance(4)
        self.wait_for(lambda: self.vision.packet()[0].sequence == 4)

    def test_switch_drops_in_flight_preview_and_control_from_previous_selection(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        self.vision.request('material')
        self.vision.request('ring')
        self.assertIsNone(self.vision.latest())
        self.assertIsNone(self.vision.packet())
        self.release.set()
        # The old camera frame predates the new selection, so neither output may return it.
        time.sleep(.05)
        self.assertFalse(self.vision.error, self.vision.error)
        self.assertIsNone(self.vision.latest())
        self.assertIsNone(self.vision.packet())
        self.camera.advance(2)
        self.wait_for(lambda: self.vision.packet() is not None)
        self.assertEqual(self.vision.packet()[0].sequence, 2)
        self.assertEqual(self.vision.latest().sequence, 2)

    def test_raw_material_uses_original_new_frames_while_jpeg_is_blocked(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        self.vision.request('material', geometry_target=1)
        pixels = np.full((200, 300, 3), (0, 0, 255), np.uint8)
        cv2.circle(pixels, (80, 100), 25, (0, 200, 150), -1)
        cv2.circle(pixels, (220, 100), 25, (25, 170, 150), -1)
        self.camera.pixels = cv2.cvtColor(pixels, cv2.COLOR_HSV2BGR)
        for sequence in range(2, 8):
            self.camera.advance(sequence)
            self.wait_for(lambda: self.vision.latest() is not None and
                          self.vision.latest().sequence == sequence)
            obs = self.vision.latest()
            self.assertEqual(obs.image_size, (300, 200))
            self.assertEqual(obs.captured_at, self.camera.frame[3])
            self.assertEqual(obs.geometry_target, 1)
            self.assertEqual({c for c, _ in obs.materials}, {1, 2})
            self.assertAlmostEqual(dict(obs.materials)[1][0], 80, delta=1)
            self.assertIsNone(self.vision.packet())
        metrics = self.vision.metrics()
        self.assertGreater(metrics['processing_fps'], 0.)
        self.assertEqual(metrics['preview_fps'], 0.)
        self.assertGreaterEqual(metrics['stages_ms']['capture_to_result'], 0.)
        self.release.set()
        self.camera.advance(8)
        self.wait_for(lambda: self.vision.packet() is not None)
        obs, jpeg = self.vision.packet()
        self.assertEqual((obs.kind, obs.sequence), ('material', 8))
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (160, 240, 3))

    def test_target_request_refines_only_selected_ring_and_discards_old_preview(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        self.vision.request('ring', geometry_target=1)
        self.camera.advance(2)
        self.wait_for(lambda: self.vision.latest().sequence == 2)
        obs = self.vision.latest()
        self.assertEqual(obs.geometry_target, 1)
        self.assertEqual(obs.center_sources[2], 'bbox')
        self.assertEqual(set(obs.rings), {1, 2, 3})
        self.release.set()
        time.sleep(.05)
        self.assertIsNone(self.vision.packet())
        self.vision.request('ring', geometry_target=2)
        self.camera.advance(3)
        self.wait_for(lambda: self.vision.packet() is not None)
        obs, _ = self.vision.packet()
        self.assertEqual((obs.sequence, obs.geometry_target), (3, 2))
        self.assertEqual(obs.center_sources, {1:'bbox', 2:'circle', 3:'bbox'})

    def test_raw_selection_applies_to_control_and_preview_and_clears_on_area_switch(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        self.release.set()
        pixels = np.full((200, 400, 3), (0, 0, 255), np.uint8)
        for x, radius, hue in ((60, 25, 0), (150, 15, 0), (240, 15, 25), (330, 25, 25)):
            cv2.circle(pixels, (x, 100), radius, (hue, 200, 150), -1)
        self.camera.pixels = cv2.cvtColor(pixels, cv2.COLOR_HSV2BGR)
        encode = self.vision._encode_preview
        results = {}

        def record_preview(pixels, result, obs, generation):
            results[obs.sequence] = result
            return encode(pixels, result, obs, generation)

        with patch.object(self.vision, '_encode_preview', record_preview):
            for sequence, largest, expected in ((2, True, [1, 2]), (3, False, [1, 1, 2, 2])):
                self.vision.request('material', largest_per_color=largest)
                self.assertIsNone(self.vision.latest())
                self.assertIsNone(self.vision.packet())
                self.camera.advance(sequence)
                self.wait_for(lambda: self.vision.packet() is not None)
                preview_obs, _ = self.vision.packet()
                obs = self.vision.latest()
                self.assertEqual(obs.materials, preview_obs.materials)
                self.assertEqual([c for c, _ in obs.materials], expected)
                self.assertEqual([o.color_id for o in results[sequence].observations], expected)
                if largest:
                    self.assertAlmostEqual(obs.materials[0][1][0], 60, delta=1)
                    self.assertAlmostEqual(obs.materials[1][1][0], 330, delta=1)

    def test_material_target_switch_discards_inflight_process_result(self):
        self.assertTrue(self.entered.wait(timeout=3.))
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        infer = self.vision._infer_next

        def pause_material(*args):
            result = infer(*args)
            if result is not None and args[-2:] == ('material', 1):
                entered.set()
                if not release.wait(3.):
                    raise RuntimeError('test did not release material result')
            return result

        with patch.object(self.vision, '_infer_next', pause_material):
            self.vision.request('material', geometry_target=1)
            self.camera.advance(2)
            self.assertTrue(entered.wait(3.))
            self.vision.request('ring')
            self.vision.request('material', geometry_target=2)
            release.set()
            time.sleep(.05)
            self.assertIsNone(self.vision.latest())
            self.assertIsNone(self.vision.packet())
            self.camera.advance(3)
            self.wait_for(lambda: self.vision.latest() is not None)
            obs = self.vision.latest()
            self.assertEqual((obs.kind, obs.sequence, obs.geometry_target), ('material', 3, 2))


class MaterialModeTests(unittest.TestCase):
    root = Path(__file__).resolve().parents[3]
    package = Path('/root/去年开源/gc_material_model_v218_20261116/detector')
    onnx = package/'material_cap_v218_384x640_9a6d18d0cc69.onnx'

    def start_vision(self, model_path):
        folder = tempfile.TemporaryDirectory(prefix='coarse_material_')
        self.addCleanup(folder.cleanup)
        material_path = Path(folder.name)/'material.yaml'
        config = yaml.safe_load((self.root/'ros2_ws/src/agv_perception/config/material.yaml').read_text())
        config['material_debug_node']['ros__parameters'].update(
            material_backend=1, use_jhb_model=False, **{'jhb.model_path':str(model_path)})
        material_path.write_text(yaml.safe_dump(config))
        self.camera = CameraFixture()
        camera_patch = patch('agv_perception.work_vision.CameraStream', return_value=self.camera)
        ring_patch = patch('agv_perception.work_vision.load_ring_model', return_value=ModelFixture())
        camera_patch.start()
        ring_patch.start()
        self.addCleanup(camera_patch.stop)
        self.addCleanup(ring_patch.stop)
        self.vision = RingVision(self.root/'ros2_ws/src/agv_perception/config/ring.yaml',
                                 240, logging.getLogger('test_material_mode'), material_path)
        self.addCleanup(self.vision.close)
        self.wait_for(lambda:self.vision.packet() is not None)

    def wait_for(self, predicate):
        deadline = time.monotonic()+10.
        while time.monotonic()<deadline:
            self.assertFalse(self.vision.error, self.vision.error)
            if predicate():
                return
            time.sleep(.005)
        self.fail('material model switch or observation timed out')

    def next_frame(self, sequence, kind):
        self.camera.advance(sequence)
        self.wait_for(lambda:self.vision.packet() is not None and
                      self.vision.packet()[0].sequence == sequence)
        self.assertEqual(self.vision.latest().kind,kind)
        self.assertTrue(self.vision.preview())

    @unittest.skipUnless(onnx.is_file(),'JHB ONNX model package not installed')
    def test_runtime_roundtrip_uses_real_jhb_and_original_detectors(self):
        self.start_vision(self.onnx)
        self.vision.request_material_model(True)  # Switch while ring preview is selected.
        self.assertIsNone(self.vision.latest())
        self.assertIsNone(self.vision.packet())
        self.wait_for(lambda:not self.vision.material_model_state()['switching'])
        self.assertEqual(self.vision.material_model_state()['info']['backend'],'jhb')
        self.vision.request('material')
        ring_pixels=self.camera.pixels
        self.camera.pixels=cv2.imread(str(self.package/'selftest.jpg'))
        self.next_frame(2,'material')
        self.assertEqual({c for c,_ in self.vision.latest().materials},{0,3,4})
        self.assertEqual(set(self.vision.latest().center_sources), {0,1,2})
        self.vision.request_material_model(False)
        self.wait_for(lambda:not self.vision.material_model_state()['switching'])
        self.assertEqual(self.vision.material_model_state()['info']['backend'],'hsv')
        self.next_frame(3,'material')
        self.vision.request('ring')
        self.camera.pixels=ring_pixels
        self.next_frame(4,'ring')
        self.assertEqual(set(self.vision.latest().rings),{1,2,3})

    def test_failed_load_preserves_original_and_resumes_fresh_frames(self):
        self.start_vision('/tmp/nonexistent_coarse_debug_jhb.onnx')
        with self.assertRaisesRegex(ValueError,'布尔值'):
            self.vision.request_material_model('true')
        self.vision.request('material')
        self.next_frame(2,'material')
        self.vision.request_material_model(True)
        self.assertIsNone(self.vision.packet())
        self.wait_for(lambda:not self.vision.material_model_state()['switching'])
        state=self.vision.material_model_state()
        self.assertFalse(state['use_jhb_model'])
        self.assertEqual(state['info']['backend'],'hsv')
        self.assertIn('切换失败',state['error'])
        self.assertTrue(state['available'])
        self.next_frame(3,'material')

    def test_combine_switch_failure_keeps_original_when_both_old_mode_flags_are_false(self):
        self.start_vision('/tmp/nonexistent_coarse_debug_jhb.onnx')
        self.vision.request('material')
        self.next_frame(2,'material')
        self.vision.request_material_model(False, test_combine=True)
        self.assertIsNone(self.vision.packet())
        self.wait_for(lambda:not self.vision.material_model_state()['switching'])
        state = self.vision.material_model_state()
        self.assertFalse(state['test_combine'])
        self.assertFalse(state['use_jhb_model'])
        self.assertEqual(state['info']['backend'], 'hsv')
        self.assertIn('切换失败', state['error'])
        self.next_frame(3, 'material')


if __name__ == '__main__':
    unittest.main()
