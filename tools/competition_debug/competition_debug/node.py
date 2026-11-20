"""One motion owner; standard HTTP/SSE and existing production hardware adapters."""

import argparse
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException

from agv_mission.task_code import parse_task_code
from agv_perception.camera import CameraStream
from route_debug.node import DebugNode, ROOT, load_parameters, preflight
from route_debug.qr_scan import QRScan
from .session import CompetitionSession
from .work import BatchWork

TOOL = Path(__file__).resolve().parents[1]


class CompetitionNode(DebugNode):
    session_type = CompetitionSession
    web_root = TOOL/'web'
    controller_name = 'competition_debug_controller'

    def __init__(self, args):
        self.args = args
        params = load_parameters(args.mission_config, 'mission_node')
        self.use_lidar = params.get('use_lidar', True)
        if type(self.use_lidar) is not bool:
            raise ValueError('mission.yaml 中 use_lidar 必须为布尔值')
        self.lidar = None
        self.qr_preview = None
        self.qr_preview_at = 0.
        self.qr_wait = ''
        self.qr_cleanup = []
        self.idle_preview = False
        self.idle_camera = ''
        self.idle_qr_camera = None
        self.idle_qr_started_at = 0.
        super().__init__(args)
        self.idle_qr_timer = self.create_timer(.2, self._preview_idle_qr)
        self.work = BatchWork(self, {area: getattr(args, area+'_config') for area in ('raw', 'coarse', 'storage')})
        if self.use_lidar:
            from .lidar import Lidar
            self.lidar = Lidar(self, self.session.cfg['service_timeout_s'])
        self.web.preview = self.preview
        self.config_display.pop('work_source', None)
        self.config_display.update(use_lidar=self.use_lidar, qr_source=args.qr_config,
            screen_source=args.screen_config, cboard_source=args.cboard_config,
            region_sources={a: getattr(args, a+'_config') for a in ('raw', 'coarse', 'storage')},
            arm_commands={a: p['arm'] for a, p in self.work.config.items()},
            camera_sources={a: p['vision'] for a, p in self.work.config.items()})

    def services_ready(self):
        return super().services_ready() and (self.lidar is None or self.lidar.ready())

    def prepare_work(self):
        if not self.work_released():
            raise ValueError('机构或雷达未确认释放')
        if self.idle_preview:
            self.release_camera()

    def open_idle_preview(self, camera='main'):
        if self.idle_preview:
            self.release_camera()
        if camera == 'qr':
            params = load_parameters(self.qr_config, 'qr_node')
            self.idle_qr_camera = CameraStream(
                params['camera_device'], params['camera_width'], params['camera_height'],
                params['camera_fps'], lambda: self.get_clock().now().to_msg(), self.get_logger(),
                buffer_size=params.get('camera_buffer_size', 4),
                dynamic_framerate=params.get('camera_dynamic_framerate', False))
            self.idle_qr_started_at = time.monotonic()
            self.idle_qr_camera.start()
        else:
            self.work.vision.open()
        self.idle_camera = camera
        self.idle_preview = True

    def _preview_idle_qr(self):
        if self.idle_qr_camera is None:
            return
        latest = self.idle_qr_camera.latest()
        if latest is not None and time.monotonic()-latest[3] <= .5:
            self.qr_frame(latest[1], None)

    def work_health(self, active):
        why = self.work.health(active) if self.work else '区域执行端未初始化'
        if why:
            return why
        if self.lidar and self.session.state == 'INITIALIZING' and self.session.run_id:
            try:
                if not self.lidar.prepare(self.session.run_id, self.session.settings['start'], self.session.settings['heading']):
                    return '等待真实雷达执行端复位及释放'
            except RuntimeError as exc:
                return str(exc)
        return ''

    def _check_graph(self):
        super()._check_graph()
        nodes = self.get_node_names_and_namespaces()
        self.graph_ok = self.graph_ok and not any(name in (
            'route_debug_controller', 'coarse_debug_controller', 'coarse_node', 'screen_node',
            'ring_debug_node', 'material_debug_node') for name, _ in nodes)
        if self.work:
            servo = self.get_publishers_info_by_topic('/C/arm_servo')
            self.graph_ok = self.graph_ok and len(servo) == 1 and servo[0].node_name == self.get_name()

    def start_batch(self, *args):
        self.work.start_batch(*args)

    def tick_work(self, odom, odom_at):
        if self.session.radar:
            return self.lidar.tick()
        return self.work.status(self.work.tick(odom, odom_at))

    def start_lidar(self, run_id):
        self.lidar.start(run_id)

    def cancel_work(self):
        if self.session.radar:
            self.lidar.stop()
        else:
            self.work.cancel()

    def work_released(self):
        return super().work_released() and (self.lidar is None or self.lidar.released())

    def release_camera(self):
        self.idle_preview = False
        self.idle_camera = ''
        if self.idle_qr_camera is not None:
            self.idle_qr_camera.close()
            self.idle_qr_camera = None
        self.qr_preview = None
        self.qr_preview_at = 0.
        if self.work:
            self.work.vision.close()

    def close(self):
        try:
            super().close()
        finally:
            self.release_camera()
            for future in self.qr_cleanup:
                future.result()

    def fault_context(self):
        task = self.work.task if self.work else None
        arm = self.work.arm if self.work else None
        return dict(run_id=self.session.run_id, navigation_request=self.session.request_id,
            executor_session=self.session.executor_id, area=self.work.area if self.work else '',
            task_id=task.task_id if task else '', state=task.state if task else '',
            confirmed=task.confirmed if task else [], reference=task.reference if task else None,
            placements=getattr(task, 'placements', {}), arm_command=arm.command_id if arm else '',
            arm_state=arm.state if arm else '', arm_reason=arm.reason if arm else '',
            diagnostic=task.diagnostic if task else {},
            vision=self.work.vision.diagnostics() if self.work else {},
            pending_service=[kind for _, kind in self.session.pending.values()])

    def start_qr(self):
        self.qr_preview = None
        self.qr_wait = ''
        self.qr = QRScan(self, self.qr_config, result_validator=self.validate_qr,
                         frame_observer=self.qr_frame, stable_detections_required=1,
                         wait_for_camera=True, background_lifecycle=True,
                         accept_delayed_result=True)

    def stop_qr(self):
        if self.qr is not None:
            self.qr_cleanup = [future for future in self.qr_cleanup if not future.done()]
            self.qr_cleanup.append(self.qr.close())
            self.qr = None

    def validate_qr(self, text):
        try:
            parse_task_code(text)
        except ValueError as exc:
            why = '任务码无效，继续等待真实扫码：'+str(exc)
            if why != self.qr_wait:
                self.session._record(why)
            self.qr_wait = why
            self.session.reason = why
            return False
        return True

    def qr_frame(self, pixels, detection):
        now = time.monotonic()
        if now-self.qr_preview_at < .2:
            return
        frame = pixels.copy()
        if detection is not None:
            cv2.polylines(frame, [detection.polygon.astype('int32')], True, (0, 255, 0), 2)
            cv2.circle(frame, tuple(round(v) for v in detection.center), 5, (0, 0, 255), -1)
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.qr_preview, self.qr_preview_at = jpeg.tobytes(), now

    def preview(self):
        if self.idle_camera == 'main':
            return self.work.vision.preview()
        if self.idle_camera == 'qr' or self.session.state == 'SCANNING':
            return self.qr_preview if time.monotonic()-self.qr_preview_at < .6 else None
        if self.session.state == 'WORKING' and not self.session.radar:
            return self.work.vision.preview()
        return None

    def _publish_web(self):
        super()._publish_web()
        state = self.web.snapshot()
        if self.idle_camera == 'qr':
            latest = self.idle_qr_camera.latest()
            if latest is None:
                detail = ('10秒内未提供图像，请检查扫码相机设备及占用' if
                          time.monotonic()-self.idle_qr_started_at > 10. else '等待新画面')
            elif time.monotonic()-latest[3] > .5:
                detail = '图像过期/断流，请检查扫码相机'
            else:
                detail = '实时画面；仅预览'
            camera = '待命扫码相机：'+detail
        elif self.idle_preview:
            camera = '待命主相机：'+(self.work.vision.error or (
                '等待新识别画面' if self.preview() is None else '实时识别画面；仅预览，无运动动作'))
        elif self.session.state == 'SCANNING':
            camera = '二维码相机：'+('等待新画面' if self.preview() is None else '实时识别标注')
        elif self.session.state == 'WORKING' and not self.session.radar:
            camera = '主相机：'+(self.work.vision.error or (
                '等待新识别画面' if self.preview() is None else '当前区域识别画面'))
        else:
            camera = ('相机已释放' if not self.work.vision.camera else
                      '主相机保留供后续区域使用，当前阶段无预览')
        state['camera'] = camera
        state['camera_manual'] = self.idle_preview
        state['camera_kind'] = (self.idle_camera if self.idle_preview else
                                'qr' if self.session.state == 'SCANNING' else 'main')
        state['camera_enabled'] = self.idle_preview or self.session.state == 'SCANNING' or (
            self.session.state == 'WORKING' and not self.session.radar)
        board = self.session.board
        board_ready = bool(state['board_fresh'] and board and all(board.get(key) for key in
                           ('connected', 'status_valid', 'pc_control', 'odometry_valid', 'motion_allowed')))
        state['hardware'] = dict(board=board_ready, planner=super().services_ready(),
            arm=not bool(self.work.health(self.session.state == 'WORKING' and not self.session.radar)),
            screen=self.screen.ser is not None, lidar='启用' if self.use_lidar else '按配置关闭')
        self.web.publish(state)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int, default=8087)
    for name, package, filename in (
        ('planner', 'agv_planning', 'planner'), ('mission', 'agv_mission', 'mission'),
        ('cboard', 'agv_cboard', 'cboard'), ('screen', 'agv_screen', 'screen'),
        ('qr', 'agv_perception', 'qr'), ('raw', 'agv_perception', 'raw'),
        ('coarse', 'agv_perception', 'coarse'), ('storage', 'agv_perception', 'storage')):
        parser.add_argument('--'+name+'-config', default=str(ROOT/'ros2_ws/src'/package/'config'/(filename+'.yaml')))
    parser.add_argument('--debug-config', default=str(ROOT/'tools/route_debug/config/debug.yaml'))
    parser.add_argument('--screen-port')
    parser.add_argument('--work-config')
    args = parser.parse_args()
    args.work_config = args.coarse_config
    if args.check:
        preflight()
        return
    rclpy.init()
    node = None
    try:
        node = CompetitionNode(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            if rclpy.ok():
                node.close()
            else:
                node.release_camera()
                node.stop_qr()
                node.work.close()
                node.web.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
