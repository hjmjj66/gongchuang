"""One live camera shared by ring/material detectors and matching JPEG previews."""

from concurrent.futures import ThreadPoolExecutor
from collections import deque
from dataclasses import fields
from pathlib import Path
import threading
import time

import cv2
import yaml

from agv_perception.camera import CameraStream
from agv_perception.coarse_control import Observation
from agv_perception.ring_detector import DetectorConfig
from agv_perception.ring_model import load_ring_model
from agv_perception.ring_geometry import GeometryConfig
from agv_perception.ring_visualization import draw_result
from agv_perception.material_config import read_material_config, parse_material_config, material_model_info
from agv_perception.material_worker import MaterialWorker
from agv_perception.material_visualization import draw_result as draw_material

from .ring_pipeline import RingGeometryWorker


class RingVision:
    def __init__(self, path, width, logger, material_path=None):
        self.path, self.width, self.logger = Path(path), width, logger
        self.material_path = material_path
        self._kind, self._generation = 'ring', 0
        self._geometry_target = 0
        self._largest_per_color = False
        self._requested_at = time.monotonic()
        self.ready = False
        self.error = ''
        self._latest = None
        self._packet = None
        self._rates = {name: deque() for name in ('capture', 'processing', 'preview')}
        self._timing = {}
        self._material_config = None
        self._pending_material_mode = None
        self._material_switching = False
        self._material_switch_error = ''
        self._next_inference_at = {'ring': 0., 'material': 0.}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='coarse_debug_vision', daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._latest

    def preview(self):
        with self._lock:
            return self._packet[1] if self._packet else None

    def packet(self):
        with self._lock:
            return self._packet

    def metrics(self):
        now = time.monotonic()
        with self._lock:
            rates = {}
            for name, samples in self._rates.items():
                rates[name+'_fps'] = (round((samples[-1][0]-samples[0][0]) /
                    (samples[-1][1]-samples[0][1]), 1)
                    if len(samples) > 1 and now-samples[-1][1] <= 1. else 0.)
            return dict(**rates, stages_ms=dict(self._timing))

    def material_model_state(self):
        with self._lock:
            config = self._material_config
            return dict(available=self.ready and config is not None,
                        use_jhb_model=config['use_jhb_model'] if config else False,
                        test_combine=config['test_combine'] if config else False,
                        switching=self._material_switching, error=self._material_switch_error,
                        info=material_model_info(config) if config else None)

    def request_material_model(self, use_jhb_model, *, test_combine=False):
        if type(use_jhb_model) is not bool or type(test_combine) is not bool:
            raise ValueError('use_jhb_model 和 test_combine 必须为布尔值')
        if use_jhb_model and test_combine:
            raise ValueError('test_combine 与 use_jhb_model 不能同时启用')
        with self._lock:
            if not self.ready or self._material_config is None:
                raise ValueError('物料识别尚未就绪')
            if self._material_switching:
                raise ValueError('正在切换物料模型，请稍候')
            if (use_jhb_model, test_combine) == (self._material_config['use_jhb_model'],
                                                self._material_config['test_combine']):
                return
            self._pending_material_mode = (use_jhb_model, test_combine)
            self._material_switching = True
            self._material_switch_error = ''
            self._generation += 1
            self._requested_at = time.monotonic()
            self._latest = self._packet = None

    def _rate(self, name, count, at):
        samples = self._rates[name]
        if not samples or count > samples[-1][0]:
            samples.append((count, at))
            while len(samples) > 2 and samples[1][1] < at-2.:
                samples.popleft()

    def request(self, kind, *, geometry_target=0, largest_per_color=False):
        with self._lock:
            if (kind, geometry_target, largest_per_color) != (
                    self._kind, self._geometry_target, self._largest_per_color):
                if (kind, largest_per_color) != (self._kind, self._largest_per_color):
                    self._latest = None
                self._kind = kind
                self._geometry_target = geometry_target
                self._largest_per_color = largest_per_color
                self._generation += 1
                self._requested_at = time.monotonic()
                self._packet = None

    def _run(self):
        camera = detector = material = None
        try:
            p = yaml.safe_load(self.path.read_text())['ring_debug_node']['ros__parameters']
            if p['mode'] != 'combined':
                raise ValueError('粗加工调试须使用 combined 圆环模型')
            cv2.setNumThreads(p['opencv_threads'])
            detector = RingGeometryWorker(
                DetectorConfig(**{f.name: p[f.name] for f in fields(DetectorConfig)}),
                GeometryConfig(**{f.name: p['geometry.'+f.name] for f in fields(GeometryConfig)}),
                p['opencv_threads'])
            model = load_ring_model(**{name: p[name] for name in (
                'model_path', 'model_format', 'class_labels', 'model_runtime_path',
                'confidence_threshold', 'nms_threshold')})
            material = None
            if self.material_path:
                m = read_material_config(self.material_path)
                for name in ('camera_device', 'camera_width', 'camera_height'):
                    if p[name] != m[name]:
                        raise ValueError(f'圆环与物料须共享同一相机配置：{name}')
                if m['debug_mode'] != 'detect' or m['input_image']:
                    raise ValueError('物料调试须使用实时 detect 模式')
                m['model_runtime_path'] = p['model_runtime_path']
                material = MaterialWorker(m)
                with self._lock:
                    self._material_config = m
            camera = CameraStream(p['camera_device'], p['camera_width'], p['camera_height'],
                                  p['camera_fps'], time.monotonic, self.logger,
                                  p['camera_buffer_size'], p['camera_dynamic_framerate'])
            camera.start()
            self.ready = True
            last_sequence = 0
            preview = pending = None
            processed = displayed = 0
            next_preview = 0.
            preview_generation = None
            with ThreadPoolExecutor(max_workers=2, thread_name_prefix='coarse_debug_stage') as pool:
                while not self._stop.is_set():
                    if preview is not None and preview.done():
                        generation, packet = preview.result()
                        with self._lock:
                            if generation == self._generation:
                                self._packet = packet
                                displayed += 1
                                self._rate('preview', displayed, time.monotonic())
                        preview = None
                    with self._lock:
                        requested_mode = self._pending_material_mode
                        self._pending_material_mode = None
                        kind, generation, requested_at = self._kind, self._generation, self._requested_at
                        geometry_target = self._geometry_target
                    if not kind:
                        self._stop.wait(.005)
                        continue
                    if requested_mode is not None:
                        # Finish any old ring inference before loading another BPU model.
                        if pending is not None:
                            pending.result()
                            pending = None
                        try:
                            candidate = parse_material_config({**m,'use_jhb_model':requested_mode[0],
                                                                'test_combine':requested_mode[1]})
                            material.configure(candidate)
                            m = candidate
                            with self._lock:
                                self._material_config = m
                        except Exception as error:
                            self.logger.error(f'物料模型切换失败：{error}')
                            with self._lock:
                                self._material_switch_error = f'切换失败，继续使用原模式：{error}'
                        with self._lock:
                            self._material_switching = False
                            self._requested_at = time.monotonic()
                            requested_at = self._requested_at
                            self._timing = {}
                            self._rates['processing'].clear()
                            self._rates['preview'].clear()
                        last_sequence = 0
                    frame = camera.latest()
                    if frame is not None:
                        with self._lock:
                            self._rate('capture', frame[0], frame[3])
                    if pending is not None:
                        if not pending.done():
                            self._stop.wait(.001)
                            continue
                        inferred = pending.result()
                        pending = None
                        if inferred is None:
                            continue
                        frame, boxes, model_ms, inferred_generation = inferred
                        if inferred_generation != generation:
                            continue
                        last_sequence = frame[0]
                    else:
                        pending = pool.submit(self._infer_next, camera, model if kind == 'ring' else material,
                                              p if kind == 'ring' else m, generation,
                                              requested_at, last_sequence, kind, geometry_target)
                        continue
                    sequence, pixels, stamp, captured = frame
                    last_sequence = sequence
                    if kind == 'ring':
                        # One model pass and one geometry pass may overlap; neither queues old frames.
                        pending = pool.submit(self._infer_next, camera, model, p, generation,
                                              requested_at, last_sequence, kind, geometry_target)
                        result = detector.detect(pixels, boxes, sequence, stamp, geometry_target)
                        result.stages_ms.update(yolo=model_ms, geometry=result.processing_ms)
                        obs = Observation('ring', sequence, captured, result.image_size,
                                          {o.ring_id: o.center_px for o in result.observations if o.valid},
                                          center_sources={o.ring_id: o.center_source for o in result.observations if o.valid},
                                          geometry_target=geometry_target)
                    else:
                        result = boxes
                        if result.error:
                            raise RuntimeError(result.error)
                        located = [o for o in result.observations if o.center_px is not None]
                        obs = Observation('material', sequence, captured, result.image_size, {},
                                          tuple((o.color_id, o.center_px) for o in located),
                                          {i: o.center_source for i, o in enumerate(located)},
                                          geometry_target=geometry_target)
                    with self._lock:
                        if generation != self._generation:
                            continue
                        self._latest = obs
                        processed += 1
                        self._rate('processing', processed, time.monotonic())
                        self._timing.update(result.stages_ms)
                        self._timing['capture_to_result'] = (time.monotonic()-captured)*1000
                    # Busy preview drops display frames; control never waits for drawing or JPEG.
                    if preview is None and (generation != preview_generation or time.monotonic() >= next_preview):
                        if generation != preview_generation:
                            next_preview = time.monotonic()
                        next_preview = max(next_preview+1/15, time.monotonic())
                        preview_generation = generation
                        preview = pool.submit(self._encode_preview, pixels, result, obs, generation)
        except Exception as error:
            self.error = str(error)
            self.ready = False
            self.logger.error(f'识别线程停止：{error}')
        finally:
            if camera is not None:
                camera.close()
            if detector is not None:
                detector.close()
            if material is not None:
                material.close()

    def _infer_next(self, camera, model, config, generation, requested_at, last_sequence, kind, target):
        while not self._stop.is_set():
            with self._lock:
                if generation != self._generation:
                    return None
                largest_per_color = self._largest_per_color
            frame = camera.latest()
            now = time.monotonic()
            if (frame is not None and frame[0] != last_sequence and frame[3] > requested_at
                    and now >= self._next_inference_at[kind]):
                self._next_inference_at[kind] = now+1/config['process_rate_hz']
                started = time.perf_counter()
                boxes = (model.infer(frame[1]) if kind == 'ring' else
                         model.detect(frame[1], frame[0], frame[2], target_color_id=0,
                                      geometry_target=target, largest_per_color=largest_per_color))
                model_ms = (time.perf_counter()-started)*1000
                return frame, boxes, model_ms, generation
            self._stop.wait(.001)
        return None

    def _encode_preview(self, pixels, result, obs, generation):
        started = time.perf_counter()
        preview = (draw_result if obs.kind == 'ring' else draw_material)(pixels, result)
        if preview.shape[1] > self.width:
            preview = cv2.resize(preview, (self.width, round(preview.shape[0]*self.width/preview.shape[1])),
                                 interpolation=cv2.INTER_NEAREST)
        ok, jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError('预览图编码失败')
        with self._lock:
            self._timing['preview'] = (time.perf_counter()-started)*1000
        # Calibration still receives coordinates from the exact frame shown in its JPEG.
        return generation, (obs, jpeg.tobytes())

    def close(self):
        self._stop.set()
        self._thread.join(timeout=15.)
