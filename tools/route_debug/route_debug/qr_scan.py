"""Run the competition QR node only while the vehicle is at the QR station."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time

import yaml
from rclpy.parameter import Parameter

from agv_mission.task_code import parse_task_code
from agv_perception.qr_node import QRNode


class QRScan:
    def __init__(self, owner, path, *, result_validator=None, frame_observer=None,
                 stable_detections_required=5, wait_for_camera=False, background_lifecycle=False,
                 accept_delayed_result=False):
        params = yaml.safe_load(Path(path).read_text(encoding='utf-8'))['qr_node']['ros__parameters']
        params.update(wait_for_mission=False, show_preview=False,
                      stable_detections_required=stable_detections_required,
                      result_topic='/tools/route_debug/qr/result',
                      center_topic='/tools/route_debug/qr/center')
        self.executor = owner.executor
        self.node = None
        self.pool = None
        options = dict(parameter_overrides=[Parameter(name, value=value) for name, value in params.items()],
                       use_global_arguments=False, result_validator=result_validator, frame_observer=frame_observer,
                       close_camera_on_result=not background_lifecycle,
                       accept_delayed_result=accept_delayed_result)
        if background_lifecycle:
            # Creating ROS entities and joining the camera must not delay control callbacks.
            self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='qr_lifecycle')
            self.startup = self.pool.submit(QRNode, **options)
        else:
            self.node = QRNode(**options)
            self.executor.add_node(self.node)
        self.started_at = time.monotonic()
        self.wait_for_camera = wait_for_camera

    def tick(self):
        if self.node is None:
            if not self.startup.done():
                return None
            self.node = self.startup.result()
            self.executor.add_node(self.node)
        if self.node.result_published:
            return parse_task_code(self.node.stable_text).raw
        if self.wait_for_camera:
            return None
        latest = self.node.camera.latest()
        if latest is None:
            if time.monotonic()-self.started_at > 10.:
                raise RuntimeError('扫码相机10秒内未提供图像，请检查二维码相机')
        elif time.monotonic()-latest[3] > .5:
            raise RuntimeError('扫码相机图像过期/断流')
        return None

    def close(self):
        if self.node is not None:
            self.executor.remove_node(self.node)
            self.node.timer.cancel()
        if self.pool is not None:
            cleanup = self.pool.submit(self._destroy)
            self.pool.shutdown(wait=False)
            return cleanup
        elif self.node is not None:
            self.node.destroy_node()

    def _destroy(self):
        self.startup.result().destroy_node()
