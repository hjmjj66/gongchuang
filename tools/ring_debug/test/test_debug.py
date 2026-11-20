"""Exercise real geometry, frame freshness, rates and the local HTTP interface."""

import base64
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen

import cv2
import numpy as np
import yaml

from agv_perception.ring_detector import RingDetector
from agv_perception.ring_model import ModelDetection
from agv_perception.ring_visualization import draw_result, BBOX_CENTER_COLOR, CIRCLE_CENTER_COLOR
from server import (FrameRate, ImageSource, RingServer, RingSession, read_config, DEFAULT_CONFIG,
                    initialize_geometry_worker)


class BoxFixture:
    """Known model-boundary input for exercising the real ROI algorithm."""
    def __init__(self, boxes):
        self.boxes = boxes
        self.calls = 0

    def infer(self, frame):
        self.calls += 1
        return self.boxes


class CameraFixture:
    device = 'test input'

    def __init__(self, latest):
        self.frame = latest
        self.closed = False

    def latest(self):
        return self.frame

    def start(self):
        pass

    def close(self):
        self.closed = True


class RingDebugTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.full((320, 480, 3), 240, np.uint8)
        cv2.circle(self.frame, (230, 150), 66, (0, 0, 0), 3, cv2.LINE_AA)
        self.model = BoxFixture([ModelDetection('2', 0.9, (150, 70, 300, 230))])
        self.detector = RingDetector(model=self.model)
        self.config, _, _ = read_config(DEFAULT_CONFIG)

    def make_session(self, image_mode=False):
        camera = CameraFixture((1, self.frame, time.time(), time.monotonic()))
        return RingSession(camera, self.detector, self.config, image_mode=image_mode)

    def wait_result(self, session):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = session.snapshot()
            if snapshot['result']:
                return snapshot
            time.sleep(0.01)
        self.fail(f'No result: {snapshot["state"]}')

    def test_rates_count_skipped_capture_sequences_and_stop_when_idle(self):
        rate = FrameRate()
        rate.observe(1, 10.0)
        rate.observe(31, 11.0)
        rate.observe(31, 11.1)
        self.assertEqual(rate.value(11.1), 30)
        self.assertEqual(rate.value(12.1), 0)
        for number in range(32, 200):
            rate.observe(number, 11 + (number - 31) / 30)
        self.assertLess(len(rate.samples), 65)

    def test_slow_processing_rate_remains_visible_between_completions(self):
        rate = FrameRate(idle_timeout=10)
        rate.observe(1, 10)
        rate.observe(2, 13)
        self.assertEqual(rate.value(14.5), 0.3)
        self.assertEqual(rate.value(24), 0)

    def test_real_geometry_draws_both_centers_without_mutating_input(self):
        original = self.frame.copy()
        result = self.detector.detect(self.frame)
        item = result.observations[1]
        self.assertEqual(item.center_source, 'circle')
        self.assertAlmostEqual(item.circle_center_px[0], 230, delta=3)
        self.assertAlmostEqual(item.circle_center_px[1], 150, delta=3)
        self.assertEqual(item.bbox_center_px, (225, 150))
        overlay = draw_result(self.frame, result)
        self.assertTrue(np.any(np.all(overlay == BBOX_CENTER_COLOR, axis=2)))
        self.assertTrue(np.any(np.all(overlay == CIRCLE_CENTER_COLOR, axis=2)))
        np.testing.assert_array_equal(original, self.frame)

    def test_worker_processes_a_sequence_once_and_publishes_matching_image_and_result(self):
        session = self.make_session(image_mode=True)
        session.start()
        try:
            payload = self.wait_result(session)
            self.assertEqual(payload['display_frame_seq'], payload['result']['frame_seq'])
            image = cv2.imdecode(np.frombuffer(base64.b64decode(payload['image_base64']), np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(image.shape, self.frame.shape)
            time.sleep(0.1)
            self.assertEqual(self.model.calls, 1)
            self.assertIsNone(payload['camera_fps'])
        finally:
            session.close()
        self.assertTrue(session.camera.closed)
        self.assertFalse(session.thread.is_alive())

    def test_spawned_geometry_pipeline_returns_matching_original_pixels(self):
        _, detector_config, geometry_config = read_config(DEFAULT_CONFIG)
        with ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                 initializer=initialize_geometry_worker,
                                 initargs=(detector_config, geometry_config, 2)) as pool:
            session = self.make_session(image_mode=True)
            session.geometry_executor = pool
            session.start()
            try:
                payload = self.wait_result(session)
                self.assertEqual(payload['display_frame_seq'], payload['result']['frame_seq'])
                item = payload['result']['observations'][1]
                self.assertEqual(item['center_source'], 'circle')
                self.assertAlmostEqual(item['circle_center_px'][0], 230, delta=3)
                self.assertAlmostEqual(item['circle_center_px'][1], 150, delta=3)
                self.assertGreater(payload['result']['stages_ms']['geometry'], 0)
                self.assertEqual(self.model.calls, 1)
                buffer_name = session.geometry_buffer.name
                moved = np.full_like(self.frame, 240)
                cv2.circle(moved, (240, 152), 66, (0, 0, 0), 3, cv2.LINE_AA)
                session.camera.frame = (2, moved, time.time(), time.monotonic())
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    payload = session.snapshot()
                    if payload['result'] and payload['result']['frame_seq'] == 2:
                        break
                    time.sleep(0.01)
                self.assertEqual(payload['display_frame_seq'], 2)
                self.assertAlmostEqual(payload['result']['observations'][1]['circle_center_px'][0],
                                       240, delta=3)
                self.assertEqual(session.geometry_buffer.name, buffer_name)
            finally:
                session.close()
            with self.assertRaises(FileNotFoundError):
                SharedMemory(name=buffer_name)

    def test_busy_encoder_does_not_block_geometry_or_queue_display_frames(self):
        entered, release = threading.Event(), threading.Event()

        class CountingDetector(RingDetector):
            completed = []

            def detect_boxes(self, *args):
                result = super().detect_boxes(*args)
                self.completed.append(result.frame_seq)
                return result

        class SlowPreview(RingSession):
            def _make_packet(self, *args):
                entered.set()
                release.wait(timeout=3)
                return super()._make_packet(*args)

        detector = CountingDetector(model=self.model)
        camera = CameraFixture((1, self.frame, time.time(), time.monotonic()))
        session = SlowPreview(camera, detector, self.config, image_mode=True)
        session.start()
        try:
            self.assertTrue(entered.wait(timeout=3))
            for sequence in (2, 3):
                camera.frame = (sequence, self.frame, time.time(), time.monotonic())
                deadline = time.monotonic() + 2
                while sequence not in detector.completed and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertIn(sequence, detector.completed)
            self.assertIsNone(session.packet)
            release.set()
            payload = self.wait_result(session)
            self.assertEqual(payload['display_frame_seq'], 1)
            self.assertEqual(payload['result']['frame_seq'], 1)
        finally:
            release.set()
            session.close()

    def test_stale_and_disconnected_camera_clear_results_and_images(self):
        session = self.make_session()
        result = self.detector.detect(self.frame, 1, time.time())
        session.packet = (result.to_dict(), b'JPEG', time.monotonic())
        session.state = 'READY'
        session.camera.frame = (1, self.frame, time.time(), time.monotonic() - 2)
        payload = session.snapshot()
        self.assertEqual(payload['state'], 'STALE_FRAME')
        self.assertIsNone(payload['result'])
        self.assertIsNone(payload['image_base64'])
        session.camera.frame = None
        self.assertEqual(session.snapshot()['state'], 'NO_FRAME')
        self.assertIsNone(session.snapshot()['result'])

    def test_stale_result_shows_fresh_raw_frame_without_old_coordinates(self):
        session = self.make_session()
        result = self.detector.detect(self.frame, 1, time.time())
        session.packet = (result.to_dict(), b'old overlay', time.monotonic() - 2)
        session.last_processing_ms = 3100.0
        session.camera.frame = (2, self.frame, time.time(), time.monotonic())
        payload = session.snapshot()
        self.assertEqual(payload['state'], 'STALE_RESULT')
        self.assertIsNone(payload['result'])
        self.assertEqual(payload['display_frame_seq'], 2)
        self.assertTrue(payload['image_base64'])
        self.assertEqual(payload['last_processing_ms'], 3100)
        self.assertIn('3100.0 ms', payload['message'])

    def test_new_frame_without_boxes_never_reuses_previous_center(self):
        session = self.make_session(image_mode=True)
        session.start()
        try:
            first = self.wait_result(session)
            self.assertIsNotNone(first['result']['observations'][1]['circle_center_px'])
            self.model.boxes = []
            session.camera.frame = (2, self.frame, time.time(), time.monotonic())
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                payload = session.snapshot()
                if payload['result'] and payload['result']['frame_seq'] == 2:
                    break
                time.sleep(0.01)
            self.assertEqual(payload['result']['frame_seq'], 2)
            self.assertTrue(all(o['center_px'] is None and o['circle_center_px'] is None
                                for o in payload['result']['observations']))
        finally:
            session.close()

    def test_image_input_and_config_reject_invalid_files(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'image.png'
            cv2.imwrite(str(path), self.frame)
            source = ImageSource(path)
            np.testing.assert_array_equal(source.latest()[1], self.frame)
            config = dict(self.config, process_rate_hz=0)
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump({'ring_debug_node': {'ros__parameters': config}}))
            with self.assertRaisesRegex(ValueError, 'process_rate_hz'):
                read_config(path)

    def test_inference_error_clears_previous_result_and_recovers(self):
        session = self.make_session(image_mode=True)
        session.start()
        try:
            self.wait_result(session)
            self.model.boxes = [ModelDetection('2', 0.9, (100, 100, 10, 10))]
            deadline = time.monotonic() + 3
            with self.assertLogs('ring_debug', level='ERROR'):
                session.camera.frame = (2, self.frame, time.time(), time.monotonic())
                while time.monotonic() < deadline:
                    payload = session.snapshot()
                    if payload['state'] == 'INFERENCE_ERROR':
                        break
                    time.sleep(0.01)
            self.assertEqual(payload['state'], 'INFERENCE_ERROR')
            self.assertIsNone(payload['result'])
            self.assertTrue(payload['image_base64'])
            self.model.boxes = []
            session.camera.frame = (3, self.frame, time.time(), time.monotonic())
            recovered = self.wait_result(session)
            self.assertEqual(recovered['result']['frame_seq'], 3)
            self.assertEqual(recovered['state'], 'READY')
        finally:
            session.close()

    def test_http_returns_atomic_frame_and_local_assets(self):
        session = self.make_session(image_mode=True)
        server = RingServer(('127.0.0.1', 0), session)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        session.start()
        thread.start()
        try:
            self.wait_result(session)
            base = f'http://127.0.0.1:{server.server_port}'
            payload = json.load(urlopen(base + '/api/frame', timeout=3))
            self.assertEqual(payload['state'], 'READY')
            self.assertEqual(payload['result']['frame_seq'], payload['display_frame_seq'])
            for path, expected in (('/', b'viewport'), ('/app.js', b'processing-fps'), ('/style.css', b'.bbox')):
                self.assertIn(expected, urlopen(base + path, timeout=3).read())
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)
            session.close()


if __name__ == '__main__':
    unittest.main()
