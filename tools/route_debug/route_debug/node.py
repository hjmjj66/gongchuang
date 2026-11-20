"""ROS owner of all motion for the standalone browser debugging entry point."""

import argparse
import math
from pathlib import Path
import queue
import socket
import time

import yaml
import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node, NodeNameNonExistentError
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from std_srvs.srv import Trigger
from agv_interfaces.msg import (
    BlockedEdges, CBoardStatus, ControlCommand, Edge, OdometryFeedback,
    RouteControl, RouteExecutionProgress,
)
from agv_interfaces.srv import GetExecutorRuntime, PlanRoute, ResetExecutor, StopExecutor
from agv_planning.planner import EDGES
from agv_planning.motion_config import MOTION_PARAMETERS, validate_motion_parameters

from .session import BUSY, Session
from .screen import Screen
from .recording import RunRecorder, SAMPLE_INTERVAL_S
from .web_server import WebServer

PREFIX = '/tools/route_debug'
ROOT = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parents[1]


def load_parameters(path, node):
    with open(path, encoding='utf-8') as stream:
        result = yaml.safe_load(stream)
    return result[node]['ros__parameters']


class DebugNode(Node):
    session_type = Session
    web_root = TOOL / 'web'
    controller_name = 'route_debug_controller'

    def __init__(self, args):
        super().__init__(self.controller_name)
        with open(args.debug_config, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        for key, value in config.items():
            if key not in ('host',) and (not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0):
                raise ValueError(f'无效调试配置：{key}')
        params = load_parameters(args.planner_config, 'route_planner_node')
        mission_params = load_parameters(args.mission_config, 'mission_node')
        screen_params = load_parameters(args.screen_config, 'screen_node')
        self.screen = Screen(args.screen_port or screen_params['serial_port'], screen_params['baudrate'])
        self.screen_error = ''
        parking_offset = mission_params.get('parking_offset_mm', 200)
        if type(parking_offset) is not int or not 0 < parking_offset <= 4294967295:
            raise ValueError('parking_offset_mm must be a positive uint32 integer in mm')
        self.work_config = args.work_config
        self.work = None
        self.qr_config = args.qr_config
        self.qr = None
        self.odom_timeout = float(params.get('odometry_timeout_s', .5))
        default = params.get('default_edge_length_mm', 1200)
        lengths = {}
        for a, b in EDGES:
            key = f'{min(a,b)}_{max(a,b)}'
            lengths[tuple(sorted((a, b)))] = params.get('edge_length_mm.' + key,
                                                      params.get('edge_length_mm', {}).get(key, default))
        self.recording_enabled = False
        self.recorder = RunRecorder(getattr(args, 'data_dir', TOOL / 'data'))
        self.planner_parameters = params.copy()
        self.progress_at = None
        self.motion_save_pending = False
        self.motion_client = self.create_client(SetParametersAtomically,
                                                '/route_planner_node/set_parameters_atomically')
        self.config_display = dict(motion_tuning=self.controller_name == 'route_debug_controller',
            source=str(Path(args.planner_config).resolve()),
            **{name: params.get(name, default) for name, (_, default) in MOTION_PARAMETERS.items()},
            odometry_timeout_s=self.odom_timeout, parking_offset_mm=parking_offset,
            work_source=str(Path(self.work_config).resolve()),
            mission_source=str(Path(args.mission_config).resolve()))
        self.pub = self.create_publisher(ControlCommand, '/C/controll', 1)
        self.lease_pub = self.create_publisher(String, PREFIX + '/heartbeat', 1)
        self.map_pub = self.create_publisher(BlockedEdges, PREFIX + '/map', 1)
        self.rpc_clients = {key: self.create_client(typ, PREFIX + '/' + key) for key, typ in
                        [('plan', PlanRoute), ('get_runtime', GetExecutorRuntime),
                         ('reset_context', ResetExecutor), ('stop', StopExecutor)]}
        self.types = dict(plan=PlanRoute, get_runtime=GetExecutorRuntime,
                          reset_context=ResetExecutor, stop=StopExecutor)
        self.session = self.session_type(self, config, lengths, parking_offset_mm=parking_offset)
        self.subscriptions_ = [
            self.create_subscription(CBoardStatus, '/C/status',
                                     self._board, 1),
            self.create_subscription(OdometryFeedback, '/C/odometry',
                                     self._odometry, 1),
            self.create_subscription(RouteControl, PREFIX + '/control', self._control, 10),
            self.create_subscription(RouteExecutionProgress, PREFIX + '/progress', self._progress, 10)]
        self.web = WebServer(args.host or config['host'], args.port if args.port is not None else config['port'],
                             self.web_root, ROOT / '屏幕截图 2026-10-15 220523.png',
                             recordings=self.recorder.directory if self.config_display['motion_tuning'] else None)
        self.web.start()
        self.runtime_pending = False
        self.graph_ok = False
        self.graph_timer = self.create_timer(.5, self._check_graph)
        self.query_timer = self.create_timer(.2, self._query)
        self.timer = self.create_timer(.02, self._tick)
        self.web_timer = self.create_timer(.1, self._publish_web)
        self.record_timer = self.create_timer(SAMPLE_INTERVAL_S, self._record_tick)
        self.get_logger().info(f'浏览器调试已就绪：本机 http://127.0.0.1:{self.web.http.server_port}')
        if (args.host or config['host']) == '0.0.0.0':
            try:
                addresses = socket.gethostbyname_ex(socket.gethostname())[2]
            except OSError:
                addresses = []
            for address in addresses:
                if not address.startswith('127.'):
                    self.get_logger().info(f'局域网地址：http://{address}:{self.web.http.server_port}')
        self.get_logger().info(f'规划配置：{self.config_display}')

    def services_ready(self):
        return all(c.service_is_ready() for c in self.rpc_clients.values())

    def prepare_work(self):
        if self.work is None:
            from .work_alignment import WorkAlignment
            self.work = WorkAlignment(self, self.work_config)
        if not self.work.released():
            raise ValueError('上次机构动作尚未确认释放，请检查后重启工具')

    def start_work(self, area, task_id, binding):
        self.work.start(area, task_id, binding)

    def pause_work(self):
        if self.work is not None:
            self.work.pause()

    def resume_work(self, duration):
        if self.work is not None:
            self.work.resume(duration)

    def tick_work(self, odom, odom_at):
        task = self.work.tick(odom, odom_at)
        labels = dict(WORK_ENTRY='执行WORK工作位', ALIGN_ENTRY='圆环水平纠偏',
                      CAPTURE_REFERENCE='记录水平纠偏后的P0', CENTER_RING2='底盘对准2号圆环', WORK_EXIT='粗加工结束恢复WORK工作位',
                      ALIGN_EXIT='粗加工结束再次纠偏', STOW='执行INIT收纳',
                      RETURN_REFERENCE='收纳后返回P0', DONE='返回P0停车完成')
        return dict(state=task.state, label=labels.get(task.state, task.state),
                    done=task.state == 'DONE', velocity=[*task.velocity[:2], math.degrees(task.velocity[2])],
                    diagnostic=task.diagnostic, entry_alignment=task.entry_alignment,
                    final_alignment=task.final_alignment,
                    reason=task.reason)

    def cancel_work(self):
        self.work.cancel()

    def work_released(self):
        return self.work is None or self.work.released()

    def start_qr(self):
        try:
            from .qr_scan import QRScan
            self.qr = QRScan(self, self.qr_config)
        except ImportError as exc:
            raise RuntimeError('扫码组件未安装或未构建：' + str(exc)) from exc

    def tick_qr(self):
        return self.qr.tick()

    def stop_qr(self):
        if self.qr is not None:
            self.qr.close()
            self.qr = None

    def show_qr(self, result):
        self.screen.show_task(result)

    def show_counts(self, pick_count, place_count):
        self.screen.show_counts(pick_count, place_count)

    def screen_start(self):
        try:
            self.screen.start()
        except (serial.SerialException, OSError) as exc:
            self.screen_error = str(exc)
            raise ValueError('显示屏串口未就绪：' + str(exc)) from exc
        self.screen_error = ''

    def screen_stop(self):
        self.screen.stop()

    def exclusive(self):
        return self.graph_ok

    def _check_graph(self):
        pubs = self.get_publishers_info_by_topic('/C/controll')
        odom = self.get_publishers_info_by_topic('/C/odometry')
        status = self.get_publishers_info_by_topic('/C/status')
        services = self.get_service_names_and_types()
        type_map = dict(services)
        service_ok = all(name in type_map for name in
                         (PREFIX + '/plan', PREFIX + '/stop', PREFIX + '/reset_context', PREFIX + '/get_runtime'))
        # Count nodes serving private services as well as endpoints publishing motion.
        nodes = self.get_node_names_and_namespaces()
        self.graph_ok = (len(pubs) == 1 and pubs[0].node_name == self.get_name()
                         and len(odom) == 1 and len(status) == 1 and service_ok
                         and sum(name == self.controller_name for name, _ in nodes) == 1
                         and sum(name == 'route_planner_node' for name, _ in nodes) == 1
                         and sum(name == 'cboard_bridge_node' for name, _ in nodes) == 1
                         and not any(name == 'mission_node' for name, _ in nodes))

    def velocity(self, vx, vy, omega, binding=None):
        session, epoch = binding or ('', 0)
        self.pub.publish(ControlCommand(vx_mps=float(vx), vy_mps=float(vy), omega_deg_s=float(omega),
                                        odom_session_id=session, motion_epoch=epoch))

    def lease(self, run_id):
        self.lease_pub.publish(String(data=run_id))

    def map(self, run_id, revision, edges):
        self.map_pub.publish(BlockedEdges(run_id=run_id, map_revision=revision,
                                         edges=[Edge(a=a, b=b) for a, b in edges]))

    def rpc(self, kind, payload, callback):
        client = self.rpc_clients[kind]
        if not client.service_is_ready():
            callback(None, '服务未连接')
            return
        request = self.types[kind].Request()
        for key, value in payload.items():
            setattr(request, key, value)
        future = client.call_async(request)

        def done(f):
            try:
                result = dict(message_to_ordereddict(f.result()))
            except Exception as exc:
                callback(None, str(exc))
            else:
                callback(result)
        future.add_done_callback(done)

    def _query(self):
        if self.runtime_pending or not self.rpc_clients['get_runtime'].service_is_ready():
            return
        self.runtime_pending = True

        def done(result, error=None):
            self.runtime_pending = False
            if not error:
                self.session.update_runtime(result['runtime'])
        self.rpc('get_runtime', {}, done)

    def _fresh(self, msg, maximum):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return 0 <= age <= maximum

    def _control(self, msg):
        self.session.feedback(dict(message_to_ordereddict(msg)),
                              self._fresh(msg, self.session.cfg['route_control_timeout_s']))

    def _board(self, msg):
        if self._fresh(msg, self.session.cfg['status_timeout_s']):
            self.session.update_board(dict(message_to_ordereddict(msg)))

    def _odometry(self, msg):
        if self._fresh(msg, self.odom_timeout):
            self.session.update_odom(dict(message_to_ordereddict(msg)))

    def _progress(self, msg):
        data = dict(message_to_ordereddict(msg))
        self.session.progress(data, self._fresh(msg, .5))
        if self.session.segment is data:
            self.progress_at = time.monotonic()

    def _check_idle_settings(self, client):
        if not self.config_display['motion_tuning']:
            raise ValueError('此入口不提供导航调参')
        if self.motion_save_pending or self.session.state in BUSY:
            raise ValueError('当前正在执行操作，请先停止再修改设置')
        if self.session.pending or self.session.phase == 'UNCONFIRMED':
            raise ValueError('尚有未确认的操作，请等待停止确认后再保存参数')
        if (self.session.state == 'READY' and self.session.owner and self.session.owner != client
                and self.session.clock() - self.session.owner_at < self.session.cfg['browser_timeout_s']):
            raise ValueError('另一页面持有控制权；本页只读，可停止')

    def _save_motion(self, values, client, key):
        self._check_idle_settings(client)
        validate_motion_parameters(values, min(self.session.lengths.values()))
        if not self.motion_client.service_is_ready():
            raise ValueError('规划器参数服务未就绪')
        values = {name: float(value) for name, value in values.items()}
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        self.motion_save_pending = True
        future = self.motion_client.call_async(request)

        def done(f):
            error = None
            try:
                result = f.result().result
                if not result.successful:
                    error = result.reason
                else:
                    self.config_display.update(values)
                    self.planner_parameters.update(values)
                    self.session._record('导航参数已保存；下一次路线使用新参数')
            except Exception as exc:
                error = '保存结果未知，请检查配置并重启工具：' + str(exc)
            self.motion_save_pending = False
            self.web.finish(key, error)
        future.add_done_callback(done)

    def _set_recording(self, payload, client):
        self._check_idle_settings(client)
        if set(payload) != {'enabled'} or type(payload['enabled']) is not bool:
            raise ValueError('数据记录开关必须为布尔值')
        self.recording_enabled = payload['enabled']

    def _start_recording(self, previous_event):
        if not self.recording_enabled or self.recorder.stream is not None:
            return
        try:
            self.recorder.start(dict(planner_parameters=self.planner_parameters,
                                     sources=self.config_display), self.session.settings, previous_event)
            self.session._record('开始记录行走数据：' + self.recorder.name)
        except (OSError, ValueError) as exc:
            self.recorder.failed(exc)
            self.session._record(self.recorder.error)

    def _record_tick(self):
        if self.recorder.stream is None:
            return
        now = time.monotonic()
        try:
            self.recorder.sample(self.session.snapshot(), dict(
                odometry_receive_s=now-self.session.odom_at if self.session.odom else None,
                control_receive_s=now-self.session.control_at if math.isfinite(self.session.control_at) else None,
                progress_receive_s=now-self.progress_at if self.progress_at is not None else None))
        except (OSError, ValueError) as exc:
            self.recorder.failed(exc)
            self.session._record(self.recorder.error)

    def _finish_recording(self, reason=None):
        try:
            self.recorder.finish(self.session.state, reason or self.session.reason)
        except (OSError, ValueError) as exc:
            self.recorder.failed(exc)
            self.session._record(self.recorder.error)

    def _tick(self):
        for _ in range(64):
            try:
                action, payload, client, key = self.web.commands.get_nowait()
            except queue.Empty:
                break
            try:
                if action == 'heartbeat':
                    self.session.heartbeat(client)
                elif action == 'set_recording':
                    self._set_recording(payload, client)
                elif action == 'save_motion':
                    self._save_motion(payload, client, key)
                    continue
                elif self.motion_save_pending and action != 'stop':
                    raise ValueError('正在保存导航参数，请稍后再开始或复位')
                else:
                    previous_event = self.session.events[-1] if self.session.events else None
                    self.session.command(action, payload, client)
                    if action == 'start':
                        self._start_recording(previous_event)
            except (ValueError, TypeError, OSError) as exc:
                self.web.finish(key, str(exc))
            else:
                self.web.finish(key)
        self.session.tick()
        if self.recorder.stream is not None and self.session.state in ('FINISHED', 'STOPPED', 'FAULT'):
            self._record_tick()
            self._finish_recording()
        try:
            self.screen.tick(self.session.screen_message)
        except (serial.SerialException, OSError) as exc:
            self.screen_error = str(exc)
            self.screen.close()
            if self.session.settings and self.session.settings['mode'] == 'screen':
                self.session.stop('显示屏通信中断：' + str(exc), fault=True)
        if self.screen.accepting and self.session.state != 'SCREEN_SETUP':
            self.screen.stop()

    def _publish_web(self):
        state = self.session.snapshot()
        state['config'] = self.config_display.copy()
        state['motion_save_pending'] = self.motion_save_pending
        state['recording'] = dict(enabled=self.recording_enabled, **self.recorder.status())
        state['screen'] = dict(port=self.screen.port, connected=self.screen.ser is not None,
                               accepting=self.screen.accepting, error=self.screen_error)
        with self.web.lock:
            state['operations'] = [dict(client=key[0], **value[1]) for key, value in
                                   list(self.web.operations.items())[-20:]]
        self.web.publish(state)

    def close(self):
        self.velocity(0, 0, 0)
        try:
            if self.session.state in ('SCREEN_SETUP', 'NAVIGATING', 'WAITING', 'SCANNING', 'WORKING', 'INITIALIZING', 'READY'):
                self.session.stop('工具退出')
                deadline = time.monotonic() + 1.5
                while time.monotonic() < deadline and self.session.state == 'STOPPING' and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=.02)
                    self.velocity(0, 0, 0)
        finally:
            self.stop_qr()
            self.screen.close()
            if self.work is not None:
                self.work.close()
            self._finish_recording('工具退出')
            self.web.close()


def preflight():
    """Read-only graph check before launching any hardware bridge."""
    rclpy.init()
    node = rclpy.create_node('route_debug_preflight')
    conflicts = set()
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for topic in ('/C/controll', '/C/odometry', '/C/status'):
            if node.get_publishers_info_by_topic(topic):
                conflicts.add(topic)
        for name, namespace in node.get_node_names_and_namespaces():
            owner = namespace.rstrip('/') + '/' + name
            if name in ('mission_node', 'route_planner_node', 'cboard_bridge_node', 'route_debug_controller',
                        'competition_debug_controller', 'coarse_debug_controller', 'coarse_node',
                        'qr_node', 'screen_node', 'material_debug_node', 'ring_debug_node', 'camera_capture'):
                conflicts.add(owner)
            # The global service graph also includes clients (e.g. Foxglove
            # keeps clients after a bridge exits). Only servers conflict.
            try:
                services = node.get_service_names_and_types_by_node(name, namespace)
            except NodeNameNonExistentError:
                continue  # The node exited during discovery.
            for service, _ in services:
                if service.startswith(PREFIX + '/') or service == '/C/reset_fault':
                    conflicts.add(f'{service} (服务端：{owner})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if conflicts:
        raise SystemExit('检测到已有控制节点或接口，请先结束相关进程再启动工具：' + ', '.join(sorted(conflicts)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--planner-config', default=str(ROOT / 'ros2_ws/src/agv_planning/config/planner.yaml'))
    parser.add_argument('--mission-config', default=str(ROOT / 'ros2_ws/src/agv_mission/config/mission.yaml'))
    parser.add_argument('--screen-config', default=str(ROOT / 'ros2_ws/src/agv_screen/config/screen.yaml'))
    parser.add_argument('--screen-port')
    parser.add_argument('--qr-config', default=str(ROOT / 'ros2_ws/src/agv_perception/config/qr.yaml'))
    parser.add_argument('--debug-config', default=str(TOOL / 'config/debug.yaml'))
    parser.add_argument('--data-dir', default=str(TOOL / 'data'))
    parser.add_argument('--work-config', default=str(ROOT / 'tools/coarse_debug/config/debug.yaml'))
    args = parser.parse_args()
    if args.check:
        preflight()
        return
    rclpy.init()
    node = None
    try:
        node = DebugNode(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            if rclpy.ok():
                node.close()
            else:
                node.stop_qr()
                node._finish_recording('工具退出')
                node.web.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
