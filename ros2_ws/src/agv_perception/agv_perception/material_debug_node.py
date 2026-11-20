"""ROS wrapper for the shared material core; no motion or business topics."""

import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from .camera import CameraStream
from .material_config import DEFAULT_PARAMETERS, parse_material_config, material_model_info
from .material_detector import MaterialDetector
from .material_model import load_material_model
from .material_visualization import draw_result, save_result


class MaterialDebugNode(Node):
    def __init__(self, **kwargs):
        super().__init__('material_debug_node', **kwargs)
        self.camera = self.timer = self.result_file = None
        self.preview_created, self.done = False,False
        self.last_sequence = self.processed = 0
        self.last_status = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        p = self._read_parameters()
        self.config = p
        cv2.setNumThreads(p['opencv_threads'])
        self.detector = MaterialDetector(p,load_material_model(p))
        self.get_logger().info(str(material_model_info(p)))
        if p['show_preview']:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise ValueError('show_preview=true 需要桌面环境；SSH 请用网页预览或 false')
            cv2.namedWindow('Material detector',cv2.WINDOW_NORMAL)
            self.preview_created = True
        if p['output_dir']:
            directory = Path(p['output_dir']).expanduser()
            directory.mkdir(parents=True,exist_ok=True)
            if not p['input_image']:
                self.result_file = (directory/'results.jsonl').open('a',encoding='utf-8')
        if p['input_image']:
            self.timer = self.create_timer(.01,self._process_image)
        else:
            self.camera = CameraStream(p['camera_device'],p['camera_width'],p['camera_height'],
                                       p['camera_fps'],lambda:self.get_clock().now().nanoseconds/1e9,
                                       self.get_logger(),p['camera_buffer_size'],p['camera_dynamic_framerate'])
            self.camera_started_at = time.monotonic()
            self.camera.start()
            self.timer = self.create_timer(1/p['process_rate_hz'],self._process_camera)

    def _read_parameters(self):
        descriptor = ParameterDescriptor(read_only=True)
        values = {}
        for name,default in DEFAULT_PARAMETERS.items():
            if isinstance(default,list) and not default:
                value = self.declare_parameter(name,Parameter.Type.INTEGER_ARRAY,descriptor).value
                values[name] = [] if value is None else list(value)
            else:
                values[name] = self.declare_parameter(name,default,descriptor).value
        return parse_material_config(values)

    def _process_image(self):
        self.timer.cancel()
        p = self.config
        frame = cv2.imread(str(Path(p['input_image']).expanduser()),cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"无法读取图片：{p['input_image']}")
        stamp = self.get_clock().now().nanoseconds/1e9
        if p['debug_mode'] == 'geometry_only':
            result = self.detector.detect_roi(frame,p['manual_roi'],p['manual_color_id'],1,stamp)
        else:
            result = self.detector.detect(frame,1,stamp)
        self._record(frame,result,True)
        if result.error:
            raise RuntimeError(result.error)
        self.done = self.done or not p['show_preview']
        if not self.done:
            self.timer = self.create_timer(.03,self._poll_preview)

    def _process_camera(self):
        p = self.config
        latest = self.camera.latest()
        if latest is None:
            self._report_state('NO_FRAME')
            if self.last_sequence == 0 and time.monotonic()-self.camera_started_at > p['camera_start_timeout_s']:
                raise RuntimeError('CAMERA_NO_FRAME: 相机在等待时间内未提供图像')
            return
        sequence,frame,stamp,received = latest
        if time.monotonic()-received > p['max_frame_age_s']:
            self._report_state('STALE_FRAME')
            return
        if sequence == self.last_sequence:
            self._poll_preview()
            return
        self.last_sequence = sequence
        result = self.detector.detect(frame,sequence,stamp)
        if time.monotonic()-received > p['max_frame_age_s']:
            self._report_state('STALE_RESULT')
            return
        self.processed += 1
        self.last_status = None
        every = p['save_every_n_frames']
        self._record(frame,result,bool(every and self.processed % every == 0))

    def _record(self, frame, result, save_images):
        payload = result.to_dict()
        payload['timestamp_source'] = 'image_read_time' if self.config['input_image'] else 'software_frame_read_completed'
        text = json.dumps(payload,ensure_ascii=False,allow_nan=False)
        self.get_logger().info(text,throttle_duration_sec=0.0 if self.config['input_image'] else 1.0)
        if self.result_file:
            self.result_file.write(text+'\n')
            self.result_file.flush()
        if self.config['output_dir'] and save_images:
            directory = Path(self.config['output_dir']).expanduser()
            if not self.config['input_image']:
                directory /= f'frame_{result.frame_seq:08d}'
            save_result(directory,frame,result)
        if self.preview_created:
            cv2.imshow('Material detector',draw_result(frame,result))
            self._poll_preview()

    def _report_state(self, status):
        self.get_logger().warning(status,throttle_duration_sec=2.0)
        if status != self.last_status and self.result_file:
            self.result_file.write(json.dumps({'status':status,'observations':[]})+'\n')
            self.result_file.flush()
        self.last_status = status
        if self.preview_created:
            # Never leave a stale annotated target visible as the current observation.
            blank = np.zeros((120,640,3),np.uint8)
            cv2.putText(blank,status,(16,65),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
            cv2.imshow('Material detector',blank)
        self._poll_preview()

    def _poll_preview(self):
        if self.preview_created and cv2.waitKey(1) & 0xff in (27,ord('q')):
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
            cv2.destroyWindow('Material detector')
            self.preview_created = False
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = MaterialDebugNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except (ValueError,RuntimeError,OSError,cv2.error) as error:
        rclpy.logging.get_logger('material_debug_node').error(str(error))
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
