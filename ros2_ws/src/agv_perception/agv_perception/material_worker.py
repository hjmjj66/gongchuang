"""Keep material inference and geometry outside the ROS/HTTP interpreter."""

from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
import time

import cv2
import numpy as np

from .material_detector import MaterialDetector
from .material_model import load_material_model


def configure_material(config):
    global DETECTOR
    try:
        cv2.setNumThreads(config['opencv_threads'])
        replacement = MaterialDetector(config, load_material_model(config))
        if replacement.model is not None:
            # Runtime postprocessing initializes lazily; finish it before live control.
            result = replacement.detect(np.zeros((config['camera_height'], config['camera_width'], 3),
                                                dtype=np.uint8))
            if result.error:
                raise RuntimeError(result.error)
    except Exception as error:
        return str(error)
    DETECTOR = replacement
    return ''


def detect_material(name, shape, sequence, stamp, color, target, largest_per_color):
    shared = SharedMemory(name=name)
    try:
        pixels = np.ndarray(shape, dtype=np.uint8, buffer=shared.buf)
        result = DETECTOR.detect(pixels, sequence, stamp, target_color_id=color,
                                 geometry_target=target, largest_per_color=largest_per_color)
    finally:
        shared.close()
    # JPEG overlays use contours, but not the large masks/gray/edge debug images.
    result.debug = {key: value for key, value in result.debug.items() if key == 'contours'}
    return result


class MaterialWorker:
    def __init__(self, config):
        self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self.buffer = None
        try:
            self.configure(config)
        except Exception:
            self.close()
            raise

    def configure(self, config):
        error = self.pool.submit(configure_material, config).result()
        if error:
            raise RuntimeError(error)

    def detect(self, frame, frame_seq=0, stamp=0., target_color_id=None, *, geometry_target=0,
               largest_per_color=False):
        started = time.perf_counter()
        if self.buffer is None or self.buffer.size < frame.nbytes:
            if self.buffer is not None:
                self.buffer.close()
                self.buffer.unlink()
            self.buffer = SharedMemory(create=True, size=frame.nbytes)
        pixels = np.ndarray(frame.shape, dtype=np.uint8, buffer=self.buffer.buf)
        np.copyto(pixels, frame)
        # One request at a time; the producer always supplies the latest capture.
        result = self.pool.submit(detect_material, self.buffer.name, frame.shape, frame_seq,
                                  stamp, target_color_id, geometry_target, largest_per_color).result()
        result.stages_ms['handoff'] = max(0., (time.perf_counter()-started)*1000-result.processing_ms)
        return result

    def close(self):
        self.pool.shutdown()
        if self.buffer is not None:
            self.buffer.close()
            self.buffer.unlink()
