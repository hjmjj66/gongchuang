"""Same-origin HTTP commands and SSE snapshots; no third-party web runtime."""

import json
import queue
import threading
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


class WebServer:
    def __init__(self, host, port, web_root, screenshot):
        self.web_root, self.screenshot = Path(web_root), Path(screenshot)
        self.commands = queue.Queue(maxsize=256)
        self.lock = threading.Lock()
        self.state = {'state': 'IDLE', 'reason': '正在初始化ROS连接'}
        self.operations = OrderedDict()
        self.closed = threading.Event()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *args):
                pass

            def respond(self, status, data, content_type='application/json; charset=utf-8'):
                body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if isinstance(data, dict) else data
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                route = urlsplit(self.path).path
                try:
                    if route == '/api/state':
                        self.respond(200, outer.snapshot())
                    elif route == '/api/events':
                        self.send_response(200)
                        self.send_header('Content-Type', 'text/event-stream')
                        self.send_header('Cache-Control', 'no-cache')
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        while not outer.closed.is_set():
                            data = json.dumps(outer.snapshot(), ensure_ascii=False, allow_nan=False)
                            self.wfile.write(('data: ' + data + '\n\n').encode())
                            self.wfile.flush()
                            outer.closed.wait(.1)
                    else:
                        files = {'/': ('index.html', 'text/html; charset=utf-8'),
                                 '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                                 '/style.css': ('style.css', 'text/css; charset=utf-8')}
                        if route == '/map.png':
                            self.respond(200, outer.screenshot.read_bytes(), 'image/png')
                        elif route in files:
                            name, mime = files[route]
                            self.respond(200, (outer.web_root / name).read_bytes(), mime)
                        else:
                            self.respond(404, {'error': '未找到资源'})
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass

            def do_POST(self):
                try:
                    origin = self.headers.get('Origin')
                    if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                        self.respond(403, {'error': '仅接受同源控制请求'})
                        return
                    if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                        self.respond(415, {'error': '需要JSON请求'})
                        return
                    size = int(self.headers.get('Content-Length', 0))
                    if not 0 < size <= 8192:
                        raise ValueError('请求大小无效')
                    payload = json.loads(self.rfile.read(size))
                    if not isinstance(payload, dict):
                        raise ValueError('请求应为JSON对象')
                    client = payload.get('client')
                    if not isinstance(client, str) or not 8 <= len(client) <= 100:
                        raise ValueError('页面身份无效')
                    route = urlsplit(self.path).path
                    if route == '/api/heartbeat':
                        outer.commands.put_nowait(('heartbeat', {}, client, None))
                        self.respond(200, {'ok': True})
                        return
                    if route != '/api/command':
                        self.respond(404, {'error': '未知接口'})
                        return
                    op = payload.get('operation_id')
                    if not isinstance(op, str) or not 8 <= len(op) <= 100:
                        raise ValueError('操作ID无效')
                    action, settings = payload.get('action'), payload.get('settings', {})
                    if not isinstance(settings, dict):
                        raise ValueError('设置无效')
                    key = (client, op)
                    fingerprint = json.dumps([action, settings], sort_keys=True)
                    with outer.lock:
                        existing = outer.operations.get(key)
                        if existing and existing[0] != fingerprint:
                            raise ValueError('操作ID不能用于不同命令')
                        if existing:
                            self.respond(200, existing[1])
                            return
                        result = {'ok': True, 'pending': True, 'operation_id': op}
                        outer.operations[key] = (fingerprint, result)
                        if len(outer.operations) > 1024:
                            outer.operations.popitem(last=False)
                    outer.commands.put_nowait((action, settings, client, key))
                    self.respond(202, result)
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    self.respond(400, {'error': str(exc)})
                except queue.Full:
                    self.respond(503, {'error': '控制请求过多'})
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.http = ThreadingHTTPServer((host, port), Handler)
        self.http.daemon_threads = True
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.state.copy()

    def publish(self, state):
        with self.lock:
            self.state = state

    def finish(self, key, error=None):
        if key:
            with self.lock:
                fingerprint, result = self.operations[key]
                result.update(ok=not error, pending=False)
                if error:
                    result['error'] = error

    def close(self):
        self.closed.set()
        self.http.shutdown()
        self.http.server_close()
        self.thread.join(2)
