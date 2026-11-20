"""CPU ring geometry isolated from ROS and overlapped with BPU inference."""

from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
import time

import cv2
import numpy as np

from agv_perception.ring_detector import RingDetector


def initialize_geometry(config, geometry, threads):
    global DETECTOR
    cv2.setNumThreads(threads)
    DETECTOR = RingDetector(config, geometry, roi_executor=ThreadPoolExecutor(max_workers=3))


def detect_geometry(name, shape, boxes, sequence, stamp, target):
    shared = SharedMemory(name=name)
    try:
        frame = np.ndarray(shape, dtype=np.uint8, buffer=shared.buf)
        result = DETECTOR.detect_boxes(frame, boxes, sequence, stamp, geometry_target=target)
    finally:
        shared.close()
    result.debug.clear()
    return result


class RingGeometryWorker:
    def __init__(self, config, geometry, threads):
        self.pool = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context('spawn'),
            initializer=initialize_geometry, initargs=(config, geometry, threads))
        self.buffer = None

    def detect(self, frame, boxes, sequence, stamp, target):
        started = time.perf_counter()
        if self.buffer is None:
            self.buffer = SharedMemory(create=True, size=frame.nbytes)
        pixels = np.ndarray(frame.shape, dtype=np.uint8, buffer=self.buffer.buf)
        np.copyto(pixels, frame)
        # Only one frame is outstanding; the next call copies the newest capture.
        result = self.pool.submit(detect_geometry, self.buffer.name, frame.shape,
                                  boxes, sequence, stamp, target).result()
        result.stages_ms['handoff'] = max(0., (time.perf_counter()-started)*1000-result.processing_ms)
        return result

    def close(self):
        self.pool.shutdown()
        if self.buffer is not None:
            self.buffer.close()
            self.buffer.unlink()
