"""Exercise real JPEG saving and HTTP endpoints with an in-memory frame source."""

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from server import CaptureServer, CaptureSession, CaptureUnavailable


class FrameSource:
    device = 'in-memory test frame (not a camera)'
    width = 1280
    height = 720

    def __init__(self):
        self.frame = np.full((self.height, self.width, 3), 235, np.uint8)
        cv2.circle(self.frame, (640, 360), 140, (20, 20, 20), 4)
        self.age = 0.0

    def latest(self):
        if self.frame is None:
            return None
        return 1, self.frame, time.time(), time.monotonic() - self.age


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.camera = FrameSource()
        self.session = CaptureSession(self.camera, self.temp.name, warmup_s=0)

    def test_saves_full_resolution_jpeg_without_changing_input(self):
        original = self.camera.frame.copy()
        saved = self.session.capture()
        image = cv2.imread(saved['path'])
        self.assertEqual(image.shape, (720, 1280, 3))
        np.testing.assert_array_equal(self.camera.frame, original)
        self.assertEqual(saved['count'], 1)
        self.assertEqual(saved['image_size'], [1280, 720])
        self.assertEqual(Path(saved['path']).parent, self.session.directory)
        self.assertRegex(saved['filename'], r'^ring_\d{8}_\d{6}_\d{6}_000001\.jpg$')
        second = self.session.capture()
        self.assertNotEqual(saved['path'], second['path'])
        self.assertEqual(len(list(self.session.directory.glob('*.jpg'))), 2)

    def test_each_launch_has_a_separate_directory(self):
        other = CaptureSession(self.camera, self.temp.name)
        self.assertNotEqual(other.directory, self.session.directory)

    def test_no_frame_or_stale_frame_does_not_save(self):
        self.camera.age = 2.0
        with self.assertRaises(CaptureUnavailable):
            self.session.capture()
        self.assertFalse(self.session.status()['ready'])
        self.camera.frame = None
        with self.assertRaises(CaptureUnavailable):
            self.session.capture()
        self.assertEqual(self.session.count, 0)
        self.assertEqual(list(self.session.directory.iterdir()), [])

    def test_warmup_prevents_early_capture(self):
        session = CaptureSession(self.camera, self.temp.name, warmup_s=2)
        self.assertFalse(session.status()['ready'])
        with self.assertRaises(CaptureUnavailable):
            session.capture()

    def test_disk_error_does_not_report_a_saved_photo(self):
        self.session.directory.rmdir()
        with self.assertRaises(OSError):
            self.session.capture()
        self.assertEqual(self.session.count, 0)
        self.assertIsNone(self.session.last_saved)
        self.assertIsNone(self.session.last_jpeg)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.camera = FrameSource()
        self.session = CaptureSession(self.camera, self.temp.name, warmup_s=0)
        self.server = CaptureServer(('127.0.0.1', 0), self.session, preview_width=320)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.server.server_port}'
        self.addCleanup(self.close_server)

    def close_server(self):
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=3)

    def test_page_capture_status_and_thumbnail(self):
        with urlopen(self.base + '/', timeout=3) as response:
            self.assertIn('主摄像头采集', response.read().decode())
        with urlopen(Request(self.base + '/api/capture', method='POST'), timeout=3) as response:
            self.assertEqual(response.status, 201)
            saved = json.load(response)
        self.assertTrue(Path(saved['path']).is_file())
        with urlopen(self.base + '/api/status', timeout=3) as response:
            status = json.load(response)
        self.assertEqual(status['count'], 1)
        self.assertEqual(status['last_saved']['filename'], saved['filename'])
        with urlopen(self.base + '/last.jpg', timeout=3) as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, (720, 1280, 3))

    def test_preview_is_smaller_but_saved_photo_stays_full_resolution(self):
        with urlopen(self.base + '/stream.mjpg', timeout=3) as response:
            self.assertEqual(response.readline(), b'--frame\r\n')
            self.assertEqual(response.readline(), b'Content-Type: image/jpeg\r\n')
            length = int(response.readline().split(b':')[1])
            self.assertEqual(response.readline(), b'\r\n')
            image = cv2.imdecode(np.frombuffer(response.read(length), np.uint8),
                                 cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, (180, 320, 3))
        saved = self.session.capture()
        self.assertEqual(cv2.imread(saved['path']).shape, (720, 1280, 3))

    def test_no_frame_returns_service_unavailable(self):
        self.camera.frame = None
        with self.assertRaises(HTTPError) as context:
            urlopen(Request(self.base + '/api/capture', method='POST'), timeout=3)
        self.assertEqual(context.exception.code, 503)
        self.assertIn('error', json.load(context.exception))
        self.assertEqual(self.session.count, 0)

    def test_unknown_path_and_unexpected_request_body_are_rejected(self):
        with self.assertRaises(HTTPError) as context:
            urlopen(self.base + '/../../server.py', timeout=3)
        self.assertEqual(context.exception.code, 404)
        with self.assertRaises(HTTPError) as context:
            urlopen(Request(self.base + '/api/capture', data=b'{}', method='POST'), timeout=3)
        self.assertEqual(context.exception.code, 400)
        self.assertEqual(self.session.count, 0)


if __name__ == '__main__':
    unittest.main()
