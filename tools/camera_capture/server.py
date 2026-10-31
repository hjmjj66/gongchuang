"""Browser preview and full-resolution snapshots, without ROS or model loading."""

import argparse
from datetime import datetime
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit

import cv2

from agv_perception.camera import CameraStream


DEFAULT_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0'
)
WEB_DIR = Path(__file__).resolve().parent / 'web'
LOG = logging.getLogger('camera_capture')


class CaptureUnavailable(RuntimeError):
    pass


class CaptureSession:
    def __init__(self, camera, output_root, prefix='ring', warmup_s=2.0):
        self.camera = camera
        self.prefix = prefix
        self.warmup_s = warmup_s
        self.first_received_at = None
        session_name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory = Path(output_root).expanduser().resolve() / session_name
        self.directory.mkdir(parents=True, exist_ok=False)
        self.lock = threading.Lock()
        self.count = 0
        self.last_saved = None
        self.last_jpeg = None

    def status(self):
        latest = self.camera.latest()
        now = time.monotonic()
        reason = self._unavailable_reason(latest, now)
        with self.lock:
            return {
                'ready': reason is None,
                'message': reason or '相机就绪',
                'device': self.camera.device,
                'requested_size': [self.camera.width, self.camera.height],
                'image_size': ([latest[1].shape[1], latest[1].shape[0]]
                               if latest is not None else None),
                'frame_age_ms': round((now - latest[3]) * 1000) if latest else None,
                'output_dir': str(self.directory),
                'count': self.count,
                'last_saved': self.last_saved,
            }

    def capture(self):
        with self.lock:
            latest = self.camera.latest()
            reason = self._unavailable_reason(latest, time.monotonic())
            if reason:
                raise CaptureUnavailable(reason)
            sequence, frame, _, _ = latest
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError('照片编码失败')
            jpeg = encoded.tobytes()
            number = self.count + 1
            saved_at = datetime.now()
            filename = f'{self.prefix}_{saved_at:%Y%m%d_%H%M%S_%f}_{number:06d}.jpg'
            path = self.directory / filename
            # Exclusive creation prevents accidental replacement of an existing photo.
            with path.open('xb') as stream:
                try:
                    stream.write(jpeg)
                except OSError:
                    path.unlink(missing_ok=True)
                    raise
            self.count = number
            self.last_jpeg = jpeg
            self.last_saved = {
                'filename': filename, 'path': str(path), 'count': number,
                'saved_at': saved_at.isoformat(timespec='milliseconds'),
                'image_size': [frame.shape[1], frame.shape[0]],
                'frame_seq': sequence,
            }
            LOG.info('已保存 %s (%sx%s)', path, frame.shape[1], frame.shape[0])
            return dict(self.last_saved)

    def _unavailable_reason(self, latest, now):
        if latest is None:
            self.first_received_at = None
            return '等待主摄像头画面，请检查设备连接及相机是否被其他程序占用'
        if now - latest[3] > 1.0:
            self.first_received_at = None
            return '相机画面已过期，暂停拍照'
        if self.first_received_at is None:
            self.first_received_at = latest[3]
        if now - self.first_received_at < self.warmup_s:
            return '相机画面稳定中，请稍候'
        return None


class CaptureServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, session, preview_width=960, preview_fps=10.0):
        self.session = session
        self.preview_width = preview_width
        self.preview_fps = preview_fps
        self.stopping = threading.Event()
        super().__init__(address, partial(CaptureHandler, directory=WEB_DIR))

    def server_close(self):
        self.stopping.set()
        super().server_close()


class CaptureHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, directory, **kwargs):
        self.web_dir = directory
        super().__init__(*args, **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
            '/style.css': ('style.css', 'text/css; charset=utf-8'),
        }
        if path in assets:
            filename, mime = assets[path]
            self._respond(200, (self.web_dir / filename).read_bytes(), mime)
        elif path == '/api/status':
            self._json(200, self.server.session.status())
        elif path == '/stream.mjpg':
            self._stream_preview()
        elif path == '/last.jpg':
            with self.server.session.lock:
                jpeg = self.server.session.last_jpeg
            if jpeg is None:
                self._json(404, {'error': '尚未拍照'})
            else:
                self._respond(200, jpeg, 'image/jpeg')
        else:
            self._json(404, {'error': '页面不存在'})

    def do_POST(self):
        if urlsplit(self.path).path != '/api/capture':
            self._json(404, {'error': '接口不存在'})
            return
        if self.headers.get('Transfer-Encoding') or self.headers.get('Content-Length', '0') != '0':
            self._json(400, {'error': '拍照请求不接受请求体'})
            return
        try:
            result = self.server.session.capture()
        except CaptureUnavailable as error:
            self._json(503, {'error': str(error)})
        except (OSError, RuntimeError, cv2.error) as error:
            LOG.error('保存照片失败：%s', error)
            self._json(500, {'error': f'保存失败：{error}'})
        else:
            self._json(201, result)

    def _stream_preview(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        last_sequence = None
        try:
            while not self.server.stopping.is_set():
                started = time.monotonic()
                latest = self.server.session.camera.latest()
                if latest is not None and latest[0] != last_sequence and started - latest[3] <= 1.0:
                    last_sequence, frame, _, _ = latest
                    height, width = frame.shape[:2]
                    if width > self.server.preview_width:
                        preview_height = max(1, round(height * self.server.preview_width / width))
                        frame = cv2.resize(frame, (self.server.preview_width, preview_height),
                                           interpolation=cv2.INTER_AREA)
                    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        data = encoded.tobytes()
                        self.wfile.write(
                            b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                            + str(len(data)).encode('ascii') + b'\r\n\r\n' + data + b'\r\n'
                        )
                        self.wfile.flush()
                self.server.stopping.wait(max(0, 1 / self.server.preview_fps
                                               - (time.monotonic() - started)))
        except (ConnectionError, OSError):
            pass

    def _json(self, status, value):
        self._respond(status, json.dumps(value, ensure_ascii=False).encode('utf-8'),
                      'application/json; charset=utf-8')

    def _respond(self, status, body, mime):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (ConnectionError, OSError):
            pass

    def log_message(self, fmt, *args):
        LOG.debug('%s %s', self.address_string(), fmt % args)


class CameraLog(logging.LoggerAdapter):
    def process(self, msg, kwargs):
        # CameraStream also supports ROS loggers, whose warning accepts this keyword.
        kwargs.pop('throttle_duration_sec', None)
        return msg, kwargs


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    camera = CameraStream(args.device, args.width, args.height, args.fps,
                          time.time, CameraLog(LOG, {}), buffer_size=4)
    try:
        session = CaptureSession(camera, args.output_dir, args.prefix)
        server = CaptureServer((args.host, args.port), session,
                               args.preview_width, args.preview_fps)
    except OSError as error:
        raise SystemExit(f'启动失败：{error}') from error
    LOG.info('保存目录：%s', session.directory)
    LOG.info('本机页面：http://127.0.0.1:%s；电脑访问 http://板子IP:%s',
             server.server_port, server.server_port)
    LOG.info('采集期间请关闭其他占用主摄像头的程序；Ctrl+C 退出。')
    camera.start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        camera.close()


def _parse_args():
    parser = argparse.ArgumentParser(description='主摄像头网页预览与数据集拍照工具')
    parser.add_argument('--device', default=DEFAULT_DEVICE, help='V4L2 主摄像头设备路径')
    parser.add_argument('--output-dir',
                        default=str(Path(__file__).resolve().parents[2] / 'ring_dataset_raw'),
                        help='板端照片根目录，默认项目根目录下的 ring_dataset_raw')
    parser.add_argument('--prefix', default='ring', help='照片名前缀，字母、数字、下划线或短横线')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--preview-width', type=int, default=960)
    parser.add_argument('--preview-fps', type=float, default=10.0)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', args.prefix):
        parser.error('--prefix 只能使用 1～40 位字母、数字、下划线或短横线')
    if not 1 <= args.port <= 65535:
        parser.error('--port 必须为 1～65535')
    if min(args.width, args.height, args.preview_width) < 1:
        parser.error('图像宽高必须为正整数')
    if any(not math.isfinite(v) or v <= 0 for v in (args.fps, args.preview_fps)):
        parser.error('帧率必须为有限正数')
    return args


if __name__ == '__main__':
    main()
