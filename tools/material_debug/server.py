"""Material preview with runtime model selection; no motion commands."""

import argparse
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import cv2
import yaml

from agv_perception.camera import CameraStream
from agv_perception.material_config import read_material_config, parse_material_config, material_model_info
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_model import load_material_model
from agv_perception.material_visualization import draw_result, save_result

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / 'ros2_ws/src/agv_perception/config/material.yaml'
WEB = Path(__file__).resolve().parent / 'web'
LOG = logging.getLogger('material_debug')


class FrameRate:
    def __init__(self):
        self.samples = deque()

    def observe(self, count, stamp):
        if not self.samples or count > self.samples[-1][0]:
            self.samples.append((count,stamp))
            while len(self.samples) > 2 and self.samples[1][1] < stamp-2:
                self.samples.popleft()

    def value(self, now):
        if len(self.samples) < 2 or now-self.samples[-1][1] > 1:
            return 0.0
        first,last = self.samples[0],self.samples[-1]
        elapsed = max(last[1],now-1)-first[1]
        return round((last[0]-first[0])/elapsed,1) if elapsed > 0 else 0.0


class ImageSource:
    def __init__(self, path):
        image = cv2.imread(str(Path(path).expanduser()), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'无法读取图片：{path}')
        self.device = str(path)
        self.frame = (1,image,time.time(),time.monotonic())

    def latest(self):
        return self.frame

    def start(self):
        pass

    def close(self):
        pass


class MaterialSession:
    def __init__(self, camera, detector, config, preview_width=1280, image_mode=False):
        self.camera, self.detector, self.config = camera, detector, config
        self.preview_width, self.image_mode = preview_width, image_mode
        self.max_age = config['max_frame_age_s']
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name='material_detection', daemon=True)
        self.capture_rate, self.process_rate = FrameRate(),FrameRate()
        self.state, self.error, self.packet = 'STARTING',None,None
        self.last_processing_ms, self.last_stages_ms = None,{}
        self.mode_revision = 0
        self.pending_mode = None
        self.switching = False
        self.switch_error = None

    def request_mode(self, use_jhb_model, *, test_combine=False):
        if type(use_jhb_model) is not bool or type(test_combine) is not bool:
            raise ValueError('use_jhb_model 和 test_combine 必须为布尔值')
        if use_jhb_model and test_combine:
            raise ValueError('test_combine 与 use_jhb_model 不能同时启用')
        with self.lock:
            if self.config['debug_mode'] == 'geometry_only':
                raise ValueError('手动几何调试不支持切换模型，请用 detect 模式启动')
            if self.switching:
                raise RuntimeError('正在切换模型，请稍候')
            if (self.config['use_jhb_model'], self.config['test_combine']) == (use_jhb_model, test_combine):
                return
            self.mode_revision += 1
            self.pending_mode, self.switching = (use_jhb_model, test_combine),True
            self.state, self.error, self.packet = 'SWITCHING',None,None
            self.switch_error = None

    def _switch_model(self, selection):
        try:
            config = parse_material_config({**self.config,'use_jhb_model':selection[0],
                                            'test_combine':selection[1]})
            detector = MaterialDetector(config,load_material_model(config))
            with self.lock:
                self.config,self.detector = config,detector
        except Exception as error:
            LOG.error('模型切换失败：%s',error)
            with self.lock:
                self.switch_error = f'切换失败，继续使用原模式：{error}'
        with self.lock:
            self.switching = False
            self.state,self.error,self.packet = 'STARTING',None,None
            self.last_processing_ms,self.last_stages_ms = None,{}
            self.process_rate = FrameRate()

    def start(self):
        self.camera.start()
        self.thread.start()

    def close(self):
        self.stopping.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=12)
        self.camera.close()

    def _run(self):
        sequence, processed, next_time = None,0,0.0
        preview = None
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix='material_preview') as pool:
            while not self.stopping.is_set():
                with self.lock:
                    requested_mode = self.pending_mode
                    self.pending_mode = None
                    revision = self.mode_revision
                if requested_mode is not None:
                    if preview is not None:
                        # Finish the old encoding before discarding it; it must never replace the new mode.
                        try:
                            preview.result()
                        except Exception:
                            pass
                        preview = None
                    self._switch_model(requested_mode)
                    sequence,next_time = None,0.0
                if preview is not None and preview.done():
                    try:
                        packet = preview.result()
                        with self.lock:
                            if self.mode_revision != revision or self.switching:
                                pass
                            elif self.image_mode or time.monotonic()-packet[2] <= self.max_age:
                                self.packet, self.state, self.error = packet,'READY',None
                            else:
                                self.packet, self.state = None,'STALE_RESULT'
                    except Exception as error:
                        self._record_error(error,revision)
                    preview = None
                latest = self.camera.latest()
                now = time.monotonic()
                if latest is not None:
                    with self.lock:
                        self.capture_rate.observe(latest[0],latest[3])
                if (latest is None or latest[0] == sequence or now < next_time
                        or (not self.image_mode and now-latest[3] > self.max_age)):
                    self.stopping.wait(.005)
                    continue
                sequence,frame,stamp,received = latest
                next_time = now + 1/self.config['process_rate_hz']
                try:
                    if self.config['debug_mode'] == 'geometry_only':
                        result = self.detector.detect_roi(frame,self.config['manual_roi'],
                                                         self.config['manual_color_id'],sequence,stamp)
                    else:
                        result = self.detector.detect(frame,sequence,stamp)
                    if result.error:
                        raise RuntimeError(result.error)
                    processed += 1
                    with self.lock:
                        if self.mode_revision != revision or self.switching:
                            continue
                        self.process_rate.observe(processed,time.monotonic())
                        self.last_processing_ms, self.last_stages_ms = result.processing_ms,result.stages_ms
                    if not self.image_mode and time.monotonic()-received > self.max_age:
                        with self.lock:
                            if self.mode_revision == revision and not self.switching:
                                self.state, self.packet = 'STALE_RESULT',None
                        if self.config['output_dir']:
                            directory = Path(self.config['output_dir']).expanduser()
                            directory.mkdir(parents=True,exist_ok=True)
                            with (directory/'results.jsonl').open('a',encoding='utf-8') as stream:
                                stream.write(json.dumps({'frame_seq':sequence,'status':'STALE_RESULT',
                                                         'observations':[]})+'\n')
                        continue
                    elif preview is None:
                        preview = pool.submit(self._make_packet,frame,result,received,revision)
                    if self.config['output_dir']:
                        if self.image_mode:
                            save_result(self.config['output_dir'],frame,result)
                        else:
                            directory = Path(self.config['output_dir']).expanduser()
                            directory.mkdir(parents=True,exist_ok=True)
                            with (directory/'results.jsonl').open('a',encoding='utf-8') as stream:
                                stream.write(json.dumps(result.to_dict(),ensure_ascii=False,allow_nan=False)+'\n')
                            every = self.config['save_every_n_frames']
                            if every and processed % every == 0:
                                save_result(directory/f'frame_{sequence:08d}',frame,result)
                except Exception as error:
                    if preview is not None:
                        preview.cancel()
                        preview = None
                    self._record_error(error,revision)

    def _make_packet(self, frame, result, received, revision=0):
        started = time.perf_counter()
        payload = result.to_dict()
        payload['timestamp_source'] = 'image_read_time' if self.image_mode else 'software_frame_read_completed'
        payload['mode_revision'] = revision
        jpeg = encode_preview(draw_result(frame,result),self.preview_width)
        payload['preview_ms'] = (time.perf_counter()-started)*1000
        return payload,jpeg,received

    def _record_error(self, error, revision=None):
        with self.lock:
            if revision is not None and revision != self.mode_revision:
                return
            if str(error) != self.error:
                LOG.error('识别失败：%s',error)
            self.state,self.error,self.packet = 'INFERENCE_ERROR',str(error),None

    def snapshot(self):
        latest = self.camera.latest()
        now = time.monotonic()
        with self.lock:
            if latest is not None:
                self.capture_rate.observe(latest[0],latest[3])
            state,error,packet = self.state,self.error,self.packet
            capture = None if self.image_mode else self.capture_rate.value(now)
            processing = None if self.image_mode else self.process_rate.value(now)
            processing_ms,stages = self.last_processing_ms,self.last_stages_ms
            info = material_model_info(self.config)
            selection = {'use_jhb_model':self.config['use_jhb_model'],
                         'test_combine':self.config['test_combine'],
                         'mode_revision':self.mode_revision,'switching':self.switching,
                         'switch_error':self.switch_error,
                         'can_switch_model':self.config['debug_mode'] != 'geometry_only'}
        camera_age = now-latest[3] if latest else None
        if selection['switching']:
            state,packet = 'SWITCHING',None
        elif latest is None:
            state,packet = 'NO_FRAME',None
        elif not self.image_mode and camera_age > self.max_age:
            state,packet = 'STALE_FRAME',None
        elif packet is not None and not self.image_mode and now-packet[2] > self.max_age:
            state,packet = 'STALE_RESULT',None
        result,jpeg = packet[:2] if packet else (None,None)
        if jpeg is None and latest is not None and (self.image_mode or camera_age <= self.max_age):
            jpeg = encode_preview(latest[1],self.preview_width)
        messages = {'READY': '图片处理完成' if self.image_mode else '识别中',
                    'SWITCHING':'正在加载所选模型，请稍候',
                    'STARTING':'等待首帧识别', 'NO_FRAME':'等待主摄像头，请检查设备或相机占用',
                    'STALE_FRAME':'相机画面已过期，中心坐标已清空',
                    'STALE_RESULT':'识别结果已过期，中心坐标已清空',
                    'INFERENCE_ERROR':f'识别失败：{error}'}
        return {'state':state,'message':messages[state], 'source':'image' if self.image_mode else 'camera',
                'device':self.camera.device, **info, **selection,
                'camera_fps':capture,'processing_fps':processing,
                'last_processing_ms':processing_ms,'last_stages_ms':stages,
                'camera_frame_seq':latest[0] if latest else None,
                'camera_frame_age_ms':round(camera_age*1000,1) if latest and not self.image_mode else None,
                'result_age_ms':round((now-packet[2])*1000,1) if packet and not self.image_mode else None,
                'display_frame_seq':result['frame_seq'] if result else latest[0] if jpeg else None,
                'image_size':[latest[1].shape[1],latest[1].shape[0]] if latest else None,
                'result':result,'image_base64':base64.b64encode(jpeg).decode('ascii') if jpeg else None}


def encode_preview(frame, preview_width):
    h,w = frame.shape[:2]
    if w > preview_width:
        frame = cv2.resize(frame,(preview_width,max(1,round(h*preview_width/w))))
    ok,encoded = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,85])
    if not ok:
        raise RuntimeError('预览图编码失败')
    return encoded.tobytes()


class MaterialServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, session):
        self.session = session
        super().__init__(address,MaterialHandler)


class MaterialHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if urlsplit(self.path).path != '/api/mode':
            self._respond(404,b'Not found','text/plain')
            return
        try:
            length = int(self.headers.get('Content-Length','0'))
            if not 0 < length <= 1024:
                raise ValueError('请求体长度无效')
            payload = json.loads(self.rfile.read(length))
            if (not isinstance(payload,dict) or 'use_jhb_model' not in payload or
                    payload.keys() - {'use_jhb_model', 'test_combine'}):
                raise ValueError('请求须包含 use_jhb_model，可选 test_combine')
            self.server.session.request_mode(payload['use_jhb_model'],
                                             test_combine=payload.get('test_combine',False))
            self._respond(202,b'{"accepted":true}','application/json')
        except (ValueError,UnicodeError) as error:
            self._respond(400,json.dumps({'error':str(error)}).encode(),'application/json')
        except RuntimeError as error:
            self._respond(409,json.dumps({'error':str(error)}).encode(),'application/json')

    def do_GET(self):
        path = urlsplit(self.path).path
        assets = {'/':('index.html','text/html; charset=utf-8'),
                  '/app.js':('app.js','text/javascript; charset=utf-8'),
                  '/style.css':('style.css','text/css; charset=utf-8')}
        if path in assets:
            filename,mime = assets[path]
            self._respond(200,(WEB/filename).read_bytes(),mime)
        elif path == '/api/frame':
            try:
                payload = self.server.session.snapshot()
                self._respond(200,json.dumps(payload,ensure_ascii=False,allow_nan=False).encode(),
                              'application/json; charset=utf-8')
            except (RuntimeError,OSError,cv2.error) as error:
                self._respond(503,json.dumps({'error':str(error)}).encode(),'application/json')
        else:
            self._respond(404,b'Not found','text/plain')

    def _respond(self, status, body, mime):
        self.send_response(status)
        self.send_header('Content-Type',mime)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (ConnectionError,OSError):
            pass

    def log_message(self, fmt, *args):
        LOG.debug(fmt,*args)


class CameraLog(logging.LoggerAdapter):
    def process(self, msg, kwargs):
        kwargs.pop('throttle_duration_sec',None)
        return msg,kwargs


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(message)s')
    session = server = None
    try:
        config = read_material_config(args.config)
        overrides = {'input_image':args.image} if args.image else {}
        if args.use_jhb_model:
            overrides.update(use_jhb_model=True, test_combine=False)
        if args.test_combine:
            overrides.update(test_combine=True, use_jhb_model=False)
        if args.jhb_model_path:
            overrides['jhb.model_path'] = args.jhb_model_path
        if args.device:
            overrides['camera_device'] = args.device
            overrides['input_image'] = ''
        config = parse_material_config({**config,**overrides})
        cv2.setNumThreads(config['opencv_threads'])
        detector = MaterialDetector(config,load_material_model(config))
        image = config['input_image']
        camera = ImageSource(image) if image else CameraStream(
            config['camera_device'],config['camera_width'],config['camera_height'],
            config['camera_fps'],time.time,CameraLog(LOG,{}),config['camera_buffer_size'],
            config['camera_dynamic_framerate'])
        session = MaterialSession(camera,detector,config,args.preview_width,bool(image))
        server = MaterialServer((args.host,args.port),session)
        session.start()
        info = material_model_info(config)
        LOG.info('物料调试：http://127.0.0.1:%s；电脑访问 http://板子IP:%s',server.server_port,server.server_port)
        LOG.info('backend=%s model=%s path=%s labels=%s；Ctrl+C 退出',
                 info['backend'],info['model_kind'],info['model_path'],info['class_labels'])
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    except (ValueError,RuntimeError,OSError,yaml.YAMLError,cv2.error) as error:
        LOG.error('启动失败：%s',error)
        return 1
    finally:
        if server is not None:
            server.server_close()
        if session is not None:
            session.close()
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description='主相机物料 HSV/顶面/整体/JHB 识别网页调试')
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--host',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=8085)
    parser.add_argument('--image',help='静态图片覆盖 YAML 的 input_image')
    parser.add_argument('--device',help='主摄像头设备覆盖 YAML')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--use-jhb-model','--use_jhb_model',action='store_true',help='启用 JHB 物料/顶帽检测、圆心定位和 Lab 六色判别')
    modes.add_argument('--test-combine','--test_combine',action='store_true',help='原 YOLO 判色 + JHB 中心，同帧融合试验')
    parser.add_argument('--jhb-model-path',help='JHB .bin 或 .onnx 路径覆盖 YAML')
    parser.add_argument('--preview-width',type=int,default=1280)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.preview_width < 1:
        parser.error('端口须为 1～65535，预览宽度须为正整数')
    if args.image and args.device:
        parser.error('--image 与 --device 不能同时使用')
    return args


if __name__ == '__main__':
    raise SystemExit(main())
