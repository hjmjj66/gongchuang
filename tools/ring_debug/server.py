"""Single-camera, single-inference browser debugger using the perception core."""

import argparse
import base64
from collections import deque
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import fields
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import yaml

from agv_perception.camera import CameraStream
from agv_perception.ring_detector import DetectorConfig, RingDetector
from agv_perception.ring_geometry import GeometryConfig
from agv_perception.ring_model import load_ring_model
from agv_perception.ring_visualization import draw_result


ROOT = Path(__file__).resolve().parents[2]
WEB = Path(__file__).resolve().parent / 'web'
DEFAULT_CONFIG = ROOT / 'ros2_ws/src/agv_perception/config/ring.yaml'
LOG = logging.getLogger('ring_debug')


def initialize_geometry_worker(detector_config, geometry_config, opencv_threads):
    # Spawn keeps the BPU runtime in the parent. The worker owns CPU geometry only.
    global GEOMETRY_DETECTOR
    cv2.setNumThreads(opencv_threads)
    roi_pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix='ring_roi')
    GEOMETRY_DETECTOR = RingDetector(detector_config, geometry_config, roi_executor=roi_pool)


def detect_geometry(buffer_name, shape, boxes, sequence, stamp):
    shared = SharedMemory(name=buffer_name)
    try:
        frame = np.ndarray(shape, dtype=np.uint8, buffer=shared.buf)
        result = GEOMETRY_DETECTOR.detect_boxes(frame, boxes, sequence, stamp)
    finally:
        shared.close()
    # Browser overlays use observations; do not copy ROI debug arrays across processes.
    result.debug.clear()
    return result


class FrameRate:
    """Sequence deltas count captured frames even when inference skips frames."""

    def __init__(self, idle_timeout=1.0):
        self.samples = deque()
        self.idle_timeout = idle_timeout

    def observe(self, count, stamp):
        if not self.samples or count > self.samples[-1][0]:
            self.samples.append((count, stamp))
            while len(self.samples) > 2 and self.samples[1][1] < stamp - 2.0:
                self.samples.popleft()

    def value(self, now):
        if len(self.samples) < 2 or now - self.samples[-1][1] > self.idle_timeout:
            return 0.0
        first, last = self.samples[0], self.samples[-1]
        elapsed = max(last[1], now - 1.0) - first[1]
        return round((last[0] - first[0]) / elapsed, 1) if elapsed > 0 else 0.0


class ImageSource:
    def __init__(self, path):
        frame = cv2.imread(str(Path(path).expanduser()), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f'无法读取图片：{path}')
        self.device = str(path)
        self.frame = (1, frame, time.time(), time.monotonic())

    def latest(self):
        return self.frame

    def start(self):
        pass

    def close(self):
        pass


class RingSession:
    def __init__(self, camera, detector, config, preview_width=1280, image_mode=False,
                 geometry_executor=None):
        self.camera = camera
        self.detector = detector
        self.geometry_executor = geometry_executor
        self.geometry_buffer = None
        self.threshold = detector.config.confidence_threshold
        self.max_age = config['max_frame_age_s']
        self.interval = 1.0 / config['process_rate_hz']
        self.preview_width = preview_width
        self.image_mode = image_mode
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name='ring_inference', daemon=True)
        self.capture_rate = FrameRate()
        self.process_rate = FrameRate(idle_timeout=10.0)
        self.last_processing_ms = None
        self.last_stages_ms = {}
        self.packet = None
        self.state = 'STARTING'
        self.error = None
        self.last_infer_sequence = None
        self.next_infer_time = 0.0

    def start(self):
        self.camera.start()
        self.thread.start()

    def close(self):
        self.stopping.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=12)
        self.camera.close()
        if self.geometry_buffer is not None:
            self.geometry_buffer.close()
            self.geometry_buffer.unlink()
            self.geometry_buffer = None

    def _run(self):
        processed = 0
        preview = None
        # The model stage waits for a NEW capture itself, so geometry cannot make
        # us miss a camera tick. Only one inference and one preview are in flight.
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix='ring_stage') as pool:
            pending = pool.submit(self._infer_next)
            while not self.stopping.is_set():
                latest = self.camera.latest()
                if latest is not None:
                    with self.lock:
                        self.capture_rate.observe(latest[0], latest[3])
                if preview is not None and preview.done():
                    self._publish_preview(preview)
                    preview = None
                if pending.done():
                    packet = pending.result()
                    if packet is None:
                        break
                    latest, boxes, model_ms, error = packet
                    sequence, frame, stamp, received = latest
                    pending = pool.submit(self._infer_next)
                    try:
                        if error is not None:
                            raise error
                        result = self._process_geometry(frame, boxes, sequence, stamp)
                        result.stages_ms = {'yolo': model_ms, 'geometry': result.processing_ms}
                        result.processing_ms += model_ms
                        finished = time.monotonic()
                        processed += 1
                        with self.lock:
                            self.process_rate.observe(processed, finished)
                            self.last_processing_ms = result.processing_ms
                            self.last_stages_ms = result.stages_ms
                        if not self.image_mode and finished - received > self.max_age:
                            with self.lock:
                                self.state, self.packet = 'STALE_RESULT', None
                        elif preview is None:
                            # A busy preview drops this display frame, never stalls
                            # recognition. Published coordinates still match its JPEG.
                            preview = pool.submit(self._make_packet, frame, result, received)
                    except Exception as error:
                        preview = None
                        self._record_error(error)
                self.stopping.wait(0.001)

    def _infer_next(self):
        while not self.stopping.is_set():
            latest = self.camera.latest()
            now = time.monotonic()
            if (latest is not None and latest[0] != self.last_infer_sequence
                    and now >= self.next_infer_time
                    and (self.image_mode or now - latest[3] <= self.max_age)):
                self.last_infer_sequence = latest[0]
                self.next_infer_time = now + self.interval
                started = time.perf_counter()
                try:
                    boxes = self.detector.model.infer(latest[1])
                    return latest, boxes, (time.perf_counter() - started) * 1000, None
                except Exception as error:
                    return latest, None, (time.perf_counter() - started) * 1000, error
            self.stopping.wait(0.001)
        return None

    def _process_geometry(self, frame, boxes, sequence, stamp):
        if self.geometry_executor is None:
            return self.detector.detect_boxes(frame, boxes, sequence, stamp)
        started = time.perf_counter()
        if self.geometry_buffer is None or self.geometry_buffer.size < frame.nbytes:
            if self.geometry_buffer is not None:
                self.geometry_buffer.close()
                self.geometry_buffer.unlink()
            self.geometry_buffer = SharedMemory(create=True, size=frame.nbytes)
        target = np.ndarray(frame.shape, dtype=np.uint8, buffer=self.geometry_buffer.buf)
        np.copyto(target, frame)
        # Exactly one geometry call is outstanding; reuse only after its result.
        # Sharing pixels avoids pickling a 2.7 MB camera frame for every job.
        result = self.geometry_executor.submit(
            detect_geometry, self.geometry_buffer.name, frame.shape, boxes, sequence, stamp).result()
        result.processing_ms = (time.perf_counter() - started) * 1000
        return result

    def _make_packet(self, frame, result, received):
        started = time.perf_counter()
        payload = result.to_dict()
        payload['timestamp_source'] = ('image_read_time' if self.image_mode
                                       else 'software_frame_read_completed')
        jpeg = encode_preview(draw_result(frame, result), self.preview_width)
        payload['preview_ms'] = (time.perf_counter() - started) * 1000
        return payload, jpeg, received

    def _publish_preview(self, future):
        try:
            packet = future.result()
            with self.lock:
                if not self.image_mode and time.monotonic() - packet[2] > self.max_age:
                    self.state, self.packet = 'STALE_RESULT', None
                else:
                    self.packet = packet
                    self.state, self.error = 'READY', None
        except Exception as error:
            self._record_error(error)

    def _record_error(self, error):
        with self.lock:
            if str(error) != self.error:
                LOG.error('识别失败：%s', error)
            self.state, self.error, self.packet = 'INFERENCE_ERROR', str(error), None

    def snapshot(self):
        latest = self.camera.latest()
        now = time.monotonic()
        with self.lock:
            if latest is not None:
                self.capture_rate.observe(latest[0], latest[3])
            packet = self.packet
            state, error = self.state, self.error
            camera_fps = self.capture_rate.value(now) if not self.image_mode else None
            processing_fps = self.process_rate.value(now) if not self.image_mode else None
            processing_ms, stages_ms = self.last_processing_ms, self.last_stages_ms
        camera_age = now - latest[3] if latest is not None else None
        if latest is None:
            state, packet = 'NO_FRAME', None
        elif not self.image_mode and camera_age > self.max_age:
            state, packet = 'STALE_FRAME', None
        elif packet is not None and not self.image_mode and now - packet[2] > self.max_age:
            state, packet = 'STALE_RESULT', None
        messages = {
            'STARTING': '模型已加载，等待首帧识别', 'READY': '识别中' if not self.image_mode else '图片处理完成',
            'NO_FRAME': '等待主摄像头，请检查设备路径或相机占用',
            'STALE_FRAME': '相机画面已过期，中心坐标已清空',
            'STALE_RESULT': (f'识别结果已过期，中心坐标已清空；最近处理 {processing_ms:.1f} ms'
                             if processing_ms is not None else '识别结果已过期，中心坐标已清空'),
            'INFERENCE_ERROR': f'识别失败：{error}',
        }
        result = packet[0] if packet else None
        jpeg = packet[1] if packet else None
        # A raw frame carries no old boxes or centers while inference is unavailable.
        if jpeg is None and latest is not None and (self.image_mode or camera_age <= self.max_age):
            jpeg = encode_preview(latest[1], self.preview_width)
        return {
            'state': state, 'message': messages[state], 'source': 'image' if self.image_mode else 'camera',
            'device': self.camera.device, 'confidence_threshold': self.threshold,
            'camera_fps': camera_fps, 'processing_fps': processing_fps,
            'last_processing_ms': processing_ms, 'last_stages_ms': stages_ms,
            'camera_frame_seq': latest[0] if latest else None,
            'camera_frame_age_ms': round(camera_age * 1000, 1) if camera_age is not None and not self.image_mode else None,
            'result_age_ms': round((now - packet[2]) * 1000, 1) if packet and not self.image_mode else None,
            'display_frame_seq': result['frame_seq'] if result else (latest[0] if jpeg else None),
            'image_size': [latest[1].shape[1], latest[1].shape[0]] if latest else None,
            'result': result,
            'image_base64': base64.b64encode(jpeg).decode('ascii') if jpeg else None,
        }


def encode_preview(frame, preview_width):
    height, width = frame.shape[:2]
    if width > preview_width:
        frame = cv2.resize(frame, (preview_width, max(1, round(height * preview_width / width))),
                           interpolation=cv2.INTER_LINEAR)
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError('预览图编码失败')
    return encoded.tobytes()


class RingServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, session):
        self.session = session
        super().__init__(address, RingHandler)


class RingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                  '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                  '/style.css': ('style.css', 'text/css; charset=utf-8')}
        if path in assets:
            filename, mime = assets[path]
            self._respond(200, (WEB / filename).read_bytes(), mime)
        elif path == '/api/frame':
            try:
                payload = self.server.session.snapshot()
                self._respond(200, json.dumps(payload, ensure_ascii=False, allow_nan=False).encode(),
                              'application/json; charset=utf-8')
            except (RuntimeError, cv2.error) as error:
                self._respond(503, json.dumps({'error': str(error)}).encode(), 'application/json')
        else:
            self._respond(404, b'Not found', 'text/plain')

    def _respond(self, code, body, mime):
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (ConnectionError, OSError):
            pass

    def log_message(self, fmt, *args):
        LOG.debug(fmt, *args)


class CameraLog(logging.LoggerAdapter):
    def process(self, msg, kwargs):
        kwargs.pop('throttle_duration_sec', None)
        return msg, kwargs


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    session = server = geometry_pool = None
    try:
        config, detector_config, geometry_config = read_config(args.config)
        cv2.setNumThreads(config.get('opencv_threads', 2))
        model = load_ring_model(config['model_path'], config['model_format'], config['class_labels'],
                                config['model_runtime_path'], config['confidence_threshold'],
                                config['nms_threshold'])
        geometry_pool = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context('spawn'),
            initializer=initialize_geometry_worker,
            initargs=(detector_config, geometry_config, config.get('opencv_threads', 2)))
        detector = RingDetector(detector_config, geometry_config, model)
        camera = ImageSource(args.image) if args.image else CameraStream(
            args.device or config['camera_device'], config['camera_width'], config['camera_height'],
            config['camera_fps'], time.time, CameraLog(LOG, {}),
            buffer_size=config['camera_buffer_size'], dynamic_framerate=config['camera_dynamic_framerate'])
        session = RingSession(camera, detector, config, args.preview_width, bool(args.image),
                              geometry_pool)
        server = RingServer((args.host, args.port), session)
        session.start()
        LOG.info('圆环调试：http://127.0.0.1:%s；电脑访问 http://板子IP:%s',
                 server.server_port, server.server_port)
        LOG.info('候选严格 > %.3f，每类最高分；Ctrl+C 退出。', detector_config.confidence_threshold)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError, OSError, yaml.YAMLError, cv2.error) as error:
        LOG.error('启动失败：%s', error)
        return 1
    finally:
        if server is not None:
            server.server_close()
        if session is not None:
            session.close()
        if geometry_pool is not None:
            geometry_pool.shutdown(wait=True)
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description='主相机 YOLO 圆环与几何圆心网页调试')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8084)
    parser.add_argument('--device', help='覆盖 YAML 中的主摄像头路径')
    parser.add_argument('--image', help='用一张真实图片运行同一模型与几何链路，替代相机')
    parser.add_argument('--preview-width', type=int, default=1280)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.preview_width < 1:
        parser.error('端口须为 1～65535，预览宽度须为正整数')
    if args.image and args.device:
        parser.error('--image 与 --device 不能同时使用')
    return args


def read_config(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('ring_debug_node'), dict):
        raise ValueError('配置须包含 ring_debug_node.ros__parameters')
    config = data['ring_debug_node'].get('ros__parameters')
    if not isinstance(config, dict):
        raise ValueError('ros__parameters 必须为参数映射')
    required = ('model_path', 'model_format', 'model_runtime_path', 'class_labels', 'nms_threshold',
                'camera_device', 'camera_width', 'camera_height', 'camera_fps', 'camera_buffer_size',
                'camera_dynamic_framerate', 'process_rate_hz', 'max_frame_age_s', 'confidence_threshold')
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f'配置缺少参数：{missing}')
    if config.get('mode', 'combined') != 'combined':
        raise ValueError('网页工具使用 combined；手动 ROI 请使用 ROS 图片调试入口')
    for name in ('model_path', 'model_format', 'model_runtime_path', 'camera_device'):
        if not isinstance(config[name], str) or not config[name].strip():
            raise ValueError(f'{name} 必须为非空字符串')
    if not isinstance(config['class_labels'], list) or sorted(config['class_labels'], key=str) != ['1', '2', '3']:
        raise ValueError('class_labels 必须为字符串 1、2、3 的模型索引顺序')
    for name in ('camera_width', 'camera_height', 'camera_buffer_size'):
        if type(config[name]) is not int or config[name] < 1:
            raise ValueError(f'{name} 必须为正整数')
    if type(config.get('opencv_threads', 2)) is not int or config.get('opencv_threads', 2) < 1:
        raise ValueError('opencv_threads 必须为正整数')
    if type(config['camera_dynamic_framerate']) is not bool:
        raise ValueError('camera_dynamic_framerate 必须为布尔值')
    for name in ('camera_fps', 'process_rate_hz', 'max_frame_age_s'):
        if type(config[name]) not in (int, float) or not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'{name} 必须为有限正数')
    detector = DetectorConfig(**{f.name: config[f.name] for f in fields(DetectorConfig) if f.name in config})
    geometry = GeometryConfig(**{f.name: config['geometry.' + f.name] for f in fields(GeometryConfig)
                                 if 'geometry.' + f.name in config})
    return config, detector, geometry


if __name__ == '__main__':
    raise SystemExit(main())
