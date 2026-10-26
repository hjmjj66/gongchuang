"""Continuously drain the USB camera and retain only its newest frame."""

import fcntl
import os
import struct
import threading
import time

import cv2

_V4L2_CID_EXPOSURE_AUTO_PRIORITY = 0x009A0903
_VIDIOC_G_CTRL = 0xC008561B
_VIDIOC_S_CTRL = 0xC008561C


def _set_dynamic_framerate(device, enabled):
    """Set V4L2 exposure auto priority and return its previous value."""
    control = bytearray(struct.pack('Ii', _V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0))
    descriptor = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    try:
        fcntl.ioctl(descriptor, _VIDIOC_G_CTRL, control, True)
        _, previous = struct.unpack('Ii', control)
        fcntl.ioctl(descriptor, _VIDIOC_S_CTRL,
                    struct.pack('Ii', _V4L2_CID_EXPOSURE_AUTO_PRIORITY, int(enabled)))
        return previous
    finally:
        os.close(descriptor)


class CameraStream:
    def __init__(self, device, width, height, fps, timestamp, logger,
                 buffer_size=4, dynamic_framerate=False):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.timestamp = timestamp
        self.logger = logger
        self.buffer_size = buffer_size
        self.dynamic_framerate = dynamic_framerate
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None
        self._sequence = 0
        self._thread = threading.Thread(target=self._run, name='qr_camera', daemon=True)

    def start(self):
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._latest

    def _clear(self):
        with self._lock:
            self._latest = None

    def _run(self):
        while not self._stop.is_set():
            capture = None
            previous_dynamic_framerate = None
            try:
                capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                if not capture.isOpened():
                    raise RuntimeError(f'Cannot open camera {self.device}')
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                capture.set(cv2.CAP_PROP_FPS, self.fps)
                if not capture.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size):
                    self.logger.warning('Camera did not accept the requested buffer count')
                try:
                    previous_dynamic_framerate = _set_dynamic_framerate(
                        self.device, self.dynamic_framerate)
                except OSError as error:
                    self.logger.warning(f'Cannot configure camera dynamic frame rate: {error}')
                announced = False
                while not self._stop.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None or not frame.size:
                        raise RuntimeError('Camera read failed')
                    stamp = self.timestamp()
                    received_at = time.monotonic()
                    if not announced:
                        height, width = frame.shape[:2]
                        self.logger.info(
                            f'Camera streaming: {self.device}, {width}x{height}; '
                            f'buffers={capture.get(cv2.CAP_PROP_BUFFERSIZE):.0f}, '
                            f'requested_fps={self.fps:g}')
                        announced = True
                    with self._lock:
                        self._sequence += 1
                        self._latest = (self._sequence, frame, stamp, received_at)
            except (RuntimeError, cv2.error) as error:
                self._clear()
                if not self._stop.is_set():
                    self.logger.warning(f'{error}; retrying in 1 second', throttle_duration_sec=5.0)
            finally:
                if capture is not None:
                    capture.release()
                if previous_dynamic_framerate is not None:
                    try:
                        _set_dynamic_framerate(self.device, previous_dynamic_framerate)
                    except OSError as error:
                        self.logger.warning(f'Cannot restore camera dynamic frame rate: {error}')
            self._stop.wait(1.0)

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            # OpenCV V4L2 reads may need up to its backend timeout to return.
            self._thread.join(timeout=12.0)
            if self._thread.is_alive():
                self.logger.warning('Camera backend is still waiting for a read to finish')
        self._clear()
