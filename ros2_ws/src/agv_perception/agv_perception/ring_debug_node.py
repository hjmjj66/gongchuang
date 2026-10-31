"""Standalone image/camera debugging; does not issue any motion commands."""

from dataclasses import fields
import json
import math
import os
from pathlib import Path
import time

import cv2
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from .camera import CameraStream
from .ring_detector import DetectorConfig, RingDetector
from .ring_geometry import GeometryConfig
from .ring_model import load_ring_model


class RingDebugNode(Node):
    def __init__(self, **kwargs):
        super().__init__('ring_debug_node', **kwargs)
        self.camera = None
        self.timer = None
        self.result_file = None
        self.preview_created = False
        self.done = False
        self.last_sequence = 0
        self.processed_count = 0
        self.last_status = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        config = self._read_parameters()
        self._validate_input(config)
        self.mode = config['mode']
        self.input_image = config['input_image']
        self.manual_roi = config['manual_roi']
        self.manual_ring_id = config['manual_ring_id'] or None
        self.show_preview = config['show_preview']
        self.output_dir = Path(config['output_dir']).expanduser() if config['output_dir'] else None
        self.save_every_n_frames = config['save_every_n_frames']
        self.max_frame_age_s = config['max_frame_age_s']
        self.camera_start_timeout_s = config['camera_start_timeout_s']
        geometry = GeometryConfig(**{
            item.name: config[f'geometry.{item.name}'] for item in fields(GeometryConfig)
        })
        detector = DetectorConfig(**{
            item.name: config[item.name] for item in fields(DetectorConfig)
        })
        model = None
        if self.mode == 'combined':
            model = load_ring_model(config['model_path'], config['model_format'],
                                    config['class_labels'])
        self.detector = RingDetector(detector, geometry, model)
        if self.show_preview:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise ValueError('show_preview=true 需要桌面显示环境；SSH 请设置为 false。')
            cv2.namedWindow('Ring detector', cv2.WINDOW_NORMAL)
            self.preview_created = True
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            if not self.input_image:
                self.result_file = (self.output_dir / 'results.jsonl').open('a', encoding='utf-8')
        if self.input_image:
            self.timer = self.create_timer(0.01, self._process_image)
        else:
            self.camera = CameraStream(
                config['camera_device'], config['camera_width'], config['camera_height'],
                config['camera_fps'], lambda: self.get_clock().now().nanoseconds / 1e9,
                self.get_logger(), buffer_size=config['camera_buffer_size'],
                dynamic_framerate=config['camera_dynamic_framerate'],
            )
            self.camera_started_at = time.monotonic()
            self.camera.start()
            self.timer = self.create_timer(1.0 / config['process_rate_hz'], self._process_camera)

    def _read_parameters(self):
        defaults = {
            'mode': 'combined', 'model_path': '', 'model_format': '',
            'class_labels': ['1', '2', '3'], 'input_image': '',
            'manual_ring_id': 0, 'show_preview': False, 'output_dir': '',
            'save_every_n_frames': 0,
            'camera_device': '/dev/v4l/by-id/usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0',
            'camera_width': 1280, 'camera_height': 720, 'camera_fps': 30.0,
            'camera_buffer_size': 4, 'camera_dynamic_framerate': False,
            'process_rate_hz': 15.0, 'max_frame_age_s': 0.5,
            'camera_start_timeout_s': 10.0,
        }
        defaults.update({item.name: getattr(DetectorConfig(), item.name)
                         for item in fields(DetectorConfig)})
        defaults.update({f'geometry.{item.name}': getattr(GeometryConfig(), item.name)
                         for item in fields(GeometryConfig)})
        descriptor = ParameterDescriptor(read_only=True)
        values = {name: self.declare_parameter(name, default, descriptor).value
                  for name, default in defaults.items()}
        # Explicit type allows an empty ROI default without guessing an array element type.
        values['manual_roi'] = self.declare_parameter(
            'manual_roi', Parameter.Type.INTEGER_ARRAY, descriptor,
        ).value
        return values

    @staticmethod
    def _validate_input(config):
        if config['mode'] not in ('combined', 'geometry_only'):
            raise ValueError('mode must be combined or geometry_only')
        if sorted(config['class_labels']) != ['1', '2', '3']:
            raise ValueError('class_labels 必须恰好包含字符串 1、2、3；顺序应与未来模型一致。')
        if config['manual_ring_id'] not in (0, 1, 2, 3):
            raise ValueError('manual_ring_id must be 0 (unspecified), 1, 2 or 3')
        if config['mode'] == 'geometry_only':
            if not config['input_image'] or config['manual_roi'] is None or len(config['manual_roi']) != 4:
                raise ValueError('geometry_only 需要 input_image 和 manual_roi: [x1,y1,x2,y2]。')
            x1, y1, x2, y2 = config['manual_roi']
            if x2 <= x1 or y2 <= y1:
                raise ValueError('manual_roi 必须有正宽高。')
        elif config['manual_roi']:
            raise ValueError('combined 模式不接受 manual_roi；手动 ROI 只能用于 geometry_only。')
        for name in ('camera_width', 'camera_height', 'camera_buffer_size'):
            if config[name] <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('camera_fps', 'process_rate_hz', 'max_frame_age_s', 'camera_start_timeout_s'):
            if not math.isfinite(config[name]) or config[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if config['save_every_n_frames'] < 0:
            raise ValueError('save_every_n_frames must be nonnegative')
        if not config['input_image'] and not config['camera_device'].strip():
            raise ValueError('camera_device must not be empty')

    def _process_image(self):
        self.timer.cancel()
        frame = cv2.imread(str(Path(self.input_image).expanduser()), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f'无法读取图片：{self.input_image}')
        stamp = self.get_clock().now().nanoseconds / 1e9
        if self.mode == 'geometry_only':
            result = self.detector.detect_roi(frame, self.manual_roi, self.manual_ring_id, 1, stamp)
        else:
            result = self.detector.detect(frame, 1, stamp)
        self._record(frame, result, 'image_read_time', save_images=True)
        self.done = self.done or not self.show_preview
        if self.show_preview and not self.done:
            self.get_logger().info('图片处理完成；预览窗口按 q 或 Esc 退出。')
            self.timer = self.create_timer(0.03, self._poll_preview)

    def _process_camera(self):
        latest = self.camera.latest()
        if latest is None:
            self._report_camera_state('NO_FRAME')
            if self.last_sequence == 0 and (
                    time.monotonic() - self.camera_started_at > self.camera_start_timeout_s):
                raise RuntimeError('CAMERA_NO_FRAME: 相机在等待时间内未提供图像。')
            return
        sequence, frame, stamp, received_at = latest
        if time.monotonic() - received_at > self.max_frame_age_s:
            self._report_camera_state('STALE_FRAME')
            return
        if sequence == self.last_sequence:
            self._poll_preview()
            return
        self.last_sequence = sequence
        self.last_status = None
        result = self.detector.detect(frame, sequence, stamp)
        if time.monotonic() - received_at > self.max_frame_age_s:
            # Do not record a long-running inference as a fresh, valid observation.
            self._report_camera_state('STALE_RESULT')
            return
        self.processed_count += 1
        save = bool(self.save_every_n_frames and
                    self.processed_count % self.save_every_n_frames == 0)
        self._record(frame, result, 'software_frame_read_completed', save)

    def _report_camera_state(self, status):
        self.get_logger().warning(status, throttle_duration_sec=2.0)
        if status != self.last_status and self.result_file is not None:
            self.result_file.write(json.dumps({'status': status, 'observations': []}) + '\n')
            self.result_file.flush()
        self.last_status = status
        self._poll_preview()

    def _record(self, frame, result, timestamp_source, save_images):
        payload = result.to_dict()
        payload['timestamp_source'] = timestamp_source
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.get_logger().info(text, throttle_duration_sec=1.0 if not self.input_image else 0.0)
        if self.result_file is not None:
            self.result_file.write(text + '\n')
            self.result_file.flush()
        if self.output_dir is not None and self.input_image:
            (self.output_dir / 'result.json').write_text(text + '\n', encoding='utf-8')
        if not self.show_preview and not (self.output_dir is not None and save_images):
            return
        preview = draw_result(frame, result)
        if self.output_dir is not None and save_images:
            prefix = 'image' if self.input_image else f'frame_{result.frame_seq:08d}'
            _save_image(self.output_dir / f'{prefix}_annotated.png', preview)
            for ring_id, geometry in result.debug.items():
                name = f'{prefix}_ring_{ring_id if ring_id is not None else "manual"}'
                _save_image(self.output_dir / f'{name}_processed.png', geometry.processed)
                _save_image(self.output_dir / f'{name}_edges.png', geometry.edges)
                roi = next(item.roi_xyxy for item in result.observations if item.ring_id == ring_id)
                x1, y1, x2, y2 = roi
                candidates = frame[y1:y2, x1:x2].copy()
                for circle in geometry.candidates:
                    color = (0, 180, 0) if circle.accepted else (0, 0, 180)
                    cv2.circle(candidates, (round(circle.x), round(circle.y)),
                               round(circle.radius), color, 1)
                _save_image(self.output_dir / f'{name}_candidates.png', candidates)
        if self.show_preview:
            cv2.imshow('Ring detector', preview)
            self._poll_preview()

    def _poll_preview(self):
        if self.show_preview and cv2.waitKey(1) & 0xff in (27, ord('q')):
            self.done = True

    def destroy_node(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self.result_file is not None:
            self.result_file.close()
            self.result_file = None
        if self.preview_created:
            cv2.destroyWindow('Ring detector')
            self.preview_created = False
        return super().destroy_node()


def draw_result(frame, result):
    preview = frame.copy()
    for item in result.observations:
        if item.bbox_xyxy is None:
            continue
        color = (0, 200, 0) if item.center_source == 'circle' else (0, 165, 255)
        if not item.valid:
            color = (0, 0, 220)
        x1, y1, x2, y2 = (round(v) for v in item.bbox_xyxy)
        cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
        if item.roi_xyxy is not None:
            rx1, ry1, rx2, ry2 = item.roi_xyxy
            cv2.rectangle(preview, (rx1, ry1), (rx2, ry2), (180, 180, 0), 1)
        label = f'{item.identity_source}:{item.ring_id} {item.center_source or item.status}'
        if item.yolo_confidence is not None:
            label += f' {item.yolo_confidence:.2f}'
        cv2.putText(preview, label, (max(0, x1), max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        if item.valid:
            center = tuple(round(v) for v in item.center_px)
            cv2.drawMarker(preview, center, color, cv2.MARKER_CROSS, 16, 2)
            for radius in item.radii_px:
                cv2.circle(preview, center, round(radius), color, 1, cv2.LINE_AA)
    return preview


def _save_image(path, frame):
    if not cv2.imwrite(str(path), frame):
        raise OSError(f'无法保存图像：{path}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = RingDebugNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, RuntimeError, OSError, cv2.error) as error:
        rclpy.logging.get_logger('ring_debug_node').error(str(error))
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
