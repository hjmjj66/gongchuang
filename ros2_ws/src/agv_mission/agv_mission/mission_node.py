"""Competition coordinator and sole publisher of the final base velocity topic."""

import math
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agv_interfaces.action import RunWorkBatch, ScanRoads
from agv_interfaces.msg import (
    BlockedEdges, CBoardStatus, ControlCommand, Edge, FirstLayerRecord, MissionState, MissionTaskInfo,
    RouteControl, RouteExecutionStatus,
)
from agv_interfaces.srv import ConfigureMission, GetExecutorRuntime, PlanRoute, ResetExecutor, StopExecutor
from agv_planning.planner import Topology

from .clients import Clients
from .context import ActiveTask, MissionContext
from .stages import LABELS, ROUTES, WORK, Stage, batch_spec, expected_steps
from .state_machine import advance, commit_steps, pause, start
from .task_code import parse_task_code
from .screen_config import parse_screen_config


class MissionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('mission_node', **kwargs)
        self.mode = self._param('mode', 'competition')
        if self.mode not in ('mission_mock', 'integration', 'competition'):
            raise ValueError('mode must be mission_mock/integration/competition')
        area = self._param('start_area', 'left')
        if area not in ('left', 'right'):
            raise ValueError('start_area must be left/right')
        finish_area = self._param('finish_area', area)
        if finish_area not in ('left', 'right'):
            raise ValueError('finish_area must be left/right')
        self.home = 1 if area == 'left' else 3
        self.finish_node = 1 if finish_area == 'left' else 3
        self.initial_heading = 'DOWN' if self.home == 1 else 'UP'
        self.parking_offset_mm = self._param('parking_offset_mm', 200)
        if type(self.parking_offset_mm) is not int or not 0 < self.parking_offset_mm <= 4294967295:
            raise ValueError('parking_offset_mm must be a positive uint32 integer in mm')
        self.screen_controlled = self._param('screen_controlled', False)
        self._screen_start_at = None
        self.response_timeout = float(self._param('response_timeout_s', 5.0))
        self.init_timeout = float(self._param('initialization_timeout_s', 15.0))
        self.runtime_max_age = float(self._param('runtime_max_age_s', 1.0))
        self.qr_input_topic = self._param('qr_input_topic', '/qr/result')
        self.qr_result_topic = self._param('qr_result_topic', '/mission/qr_result')
        if not self.qr_input_topic.strip() or not self.qr_result_topic.strip():
            raise ValueError('QR topic names must not be empty')
        rate = float(self._param('state_publish_rate_hz', 2.0))
        if not all(math.isfinite(v) and v > 0 for v in (self.response_timeout, self.init_timeout, self.runtime_max_age, rate)):
            raise ValueError('timeouts and publish rate must be finite and positive')
        self.expected_source = {}
        for key in ('route', 'lidar', 'work'):
            default = 'REAL' if self.mode == 'competition' else 'MOCK'
            source = self._param(f'{key}_source', default)
            if source not in ('REAL', 'MOCK') or (self.mode != 'integration' and source != default):
                raise ValueError('Execution sources do not match mode')
            self.expected_source[key] = source
        self.control_timeout = float(self._param('route_control_timeout_s', 0.3))
        control_rate = float(self._param('control_rate_hz', 50.0))
        if not all(math.isfinite(v) and v > 0 for v in (self.control_timeout, control_rate)):
            raise ValueError('control timeout and rate must be finite and positive')
        simulated_route = self.expected_source['route'] == 'MOCK'
        route_topic = self._param('route_control_topic',
                                  '/simulation/route/control' if simulated_route else '/route/control')
        control_topic = self._param('control_topic',
                                    '/simulation/control' if simulated_route else '/C/controll')
        if not route_topic.strip() or not control_topic.strip():
            raise ValueError('control topic names must not be empty')
        self._real_chassis = not simulated_route
        self._board_binding = None
        self._board_received = None
        self._board_ok = False
        self._board_timeout = float(self._param('board_status_timeout_s', 0.2))
        if not math.isfinite(self._board_timeout) or self._board_timeout <= 0:
            raise ValueError('board_status_timeout_s must be finite and positive')
        self.ctx = MissionContext(home_node=self.home,
                                  state='WAIT_SETUP' if self.screen_controlled else 'INITIALIZING')
        self.task_clients = Clients(self)
        self.runtime = {}
        self.sessions = {}
        self.pending = []
        self.inflight = set()
        self.reset_ids = {}
        self.reset_done = set()
        self.init_started = time.monotonic()
        self.next_poll = self.next_map = 0.0
        self._enter_pending = False
        self._stop_requested = set()
        self._stop_epoch = 0.0
        self._map_task = None
        self._map_started_at = 0.0
        self._route_control = None
        self._route_control_received = None
        self._route_control_sequence = 0
        self._route_control_wait_started = None
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(MissionState, '/mission/state', durable)
        self.info_pub = self.create_publisher(MissionTaskInfo, '/mission/task_info', durable)
        qr_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.qr_result_pub = self.create_publisher(String, self.qr_result_topic, durable)
        if self.qr_result_pub.topic_name == self.resolve_topic_name(self.qr_input_topic):
            raise ValueError('qr_input_topic and qr_result_topic must be different')
        self.map_pub = self.create_publisher(BlockedEdges, '/map/blocked_edges', 1)
        self.heartbeat = self.create_publisher(String, '/mission/heartbeat', 1)
        self.qr_sub = self.create_subscription(String, self.qr_input_topic, self._on_qr, qr_qos)
        self.board_sub = (self.create_subscription(CBoardStatus, '/C/status', self._on_board, 1)
                          if self._real_chassis else None)
        self.route_sub = self.create_subscription(RouteExecutionStatus, '/route/execution_status', self._on_route, 10)
        control_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.control_pub = self.create_publisher(ControlCommand, control_topic, control_qos)
        self.control_sub = self.create_subscription(RouteControl, route_topic, self._on_route_control, control_qos)
        if (self.control_pub.topic_name == self.control_sub.topic_name
                or self.control_sub.topic_name == '/C/controll'):
            raise ValueError('Route feedback and final base control must use different topics')
        if simulated_route and self.control_pub.topic_name == '/C/controll':
            raise ValueError('MOCK route velocities must not be forwarded to /C/controll')
        self.command_services = [self.create_service(Trigger, '/mission/' + name, handler)
                         for name, handler in (('start', self._start), ('pause', self._pause), ('reset', self._reset))]
        self.configure_service = self.create_service(
            ConfigureMission, '/mission/configure_and_start', self._configure_and_start)
        self.timer = self.create_timer(0.05, self._tick)
        self.state_timer = self.create_timer(1.0 / rate, self._publish)
        self.control_timer = self.create_timer(1.0 / control_rate, self._maintain_control)
        self._stop_control()
        self._publish()
        if self.screen_controlled:
            self.get_logger().info('WAIT_SETUP：等待屏幕确认禁行路径和起终点')

    def _param(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def _on_board(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= self._board_timeout:
            return
        binding = (message.odom_session_id, message.motion_epoch)
        changed = self._board_binding is not None and self._board_binding != binding
        self._board_binding = binding
        self._board_received = time.monotonic()-age
        self._board_ok = bool(message.motion_allowed and message.origin_initialized)
        if self.ctx.state in ('READY', 'RUNNING') and (changed or not self._board_ok):
            self.ctx.position_known = False
            self._fault('C板控制权/坐标变化或反馈异常，旧任务终止；重新摆车并初始化')

    def _chassis_ready(self):
        return (not self._real_chassis or (self._board_ok and self._board_received is not None
                and time.monotonic()-self._board_received <= self._board_timeout))

    def _stop_control(self):
        """Drop the cached command before publishing zero; never replay it after a handoff."""
        self._route_control = None
        self._route_control_received = None
        self.control_pub.publish(ControlCommand())

    def _on_route_control(self, feedback):
        c, task = self.ctx, self.ctx.active
        if (c.state != 'RUNNING' or not c.matches(feedback.run_id, feedback.request_id)
                or task.kind != 'route' or task.terminal is not None):
            return
        if (feedback.source != self.expected_source['route']
                or feedback.executor_session_id != self.sessions.get('route')):
            self._fault('路线速度反馈来源或执行进程身份冲突')
            return
        if feedback.sequence <= self._route_control_sequence:
            return
        if not self._route_control_fresh(feedback):
            return  # Queued/stale messages cannot renew the receive watchdog.
        command = feedback.command
        if self._real_chassis and (not self._chassis_ready() or
                (command.odom_session_id, command.motion_epoch) != self._board_binding):
            self._fault('路线速度绑定的坐标/控制许可已过期')
            return
        velocities = (command.vx_mps, command.vy_mps, command.omega_deg_s)
        if not all(math.isfinite(v) for v in velocities):
            self._fault('路线速度包含无效数值')
            return
        if feedback.state not in ('EXECUTING', 'COMPLETED', 'ABORTED'):
            self._fault('未知路线控制状态')
            return
        if feedback.state != 'EXECUTING' and any(v != 0.0 for v in velocities):
            self._fault('路线终态包含非零速度')
            return
        self._route_control_sequence = feedback.sequence
        if feedback.state != 'EXECUTING':
            task.terminal = feedback.state
            self._stop_control()
            if feedback.state == 'ABORTED':
                self._fault('路线执行中止：' + feedback.reason)
            return
        self._route_control = feedback
        self._route_control_received = time.monotonic()
        # A control sample can precede the asynchronous plan response. Buffer it
        # until acceptance, so unknown/rejected requests cannot move the vehicle.
        if task.accepted:
            if not self._snapshots_ready():
                self._fault('执行端运行状态过期，停止转发速度')
                return
            self.control_pub.publish(command)

    def _route_control_fresh(self, feedback):
        age = (self.get_clock().now().nanoseconds
               - feedback.header.stamp.sec * 1_000_000_000
               - feedback.header.stamp.nanosec) / 1e9
        return 0 <= age <= self.control_timeout

    def _maintain_control(self):
        c, task = self.ctx, self.ctx.active
        if (c.state != 'RUNNING' or not task or task.kind != 'route' or task.terminal is not None):
            self._stop_control()
            return
        if not task.accepted:
            self.control_pub.publish(ControlCommand())
            return
        baseline = self._route_control_received or self._route_control_wait_started
        if baseline is not None and time.monotonic() - baseline > self.control_timeout:
            self._fault('路线速度反馈超时，停止导航')
        # Fresh samples are forwarded by the subscription, without extending a
        # nonzero command's life by repeatedly publishing a cached velocity.

    def _track(self, key, future, callback, timeout=None, mutation=False):
        self.inflight.add(key)
        self.pending.append({'key': key, 'future': future, 'callback': callback,
                             'deadline': None if timeout is False else time.monotonic() + (timeout or self.response_timeout),
                             'timed_out': False, 'mutation': mutation, 'run': self.ctx.run_id})

    def _process_pending(self):
        old, self.pending = self.pending, []
        for call in old:
            future = call['future']
            if not future.done():
                if call['deadline'] is not None and time.monotonic() > call['deadline'] and not call['timed_out']:
                    call['timed_out'] = True
                    self._fault(f"{call['key']}响应超时，结果未知，未重发")
                self.pending.append(call)
                continue
            self.inflight.discard(call['key'])
            if call['run'] != self.ctx.run_id:
                continue
            if call['mutation'] and self.ctx.state in ('PAUSING', 'FAULT'):
                self._stop_epoch = time.monotonic()
            try:
                response = future.result()
                if response is None:
                    raise RuntimeError('empty response')
                call['callback'](response)
            except Exception as exc:
                self._fault(f"{call['key']}: {exc}")

    def _tick(self):
        if self.ctx.state in ('INITIALIZING', 'RESETTING', 'READY', 'RUNNING'):
            self.heartbeat.publish(String(data=self.ctx.run_id))
        self._process_pending()
        now = time.monotonic()
        if now >= self.next_poll:
            self.next_poll = now + 0.2
            for key, client in self.task_clients.get.items():
                tag = 'get:' + key
                if tag not in self.inflight and client.service_is_ready():
                    issued_at = time.monotonic()
                    self._track(tag, client.call_async(GetExecutorRuntime.Request()),
                                lambda response, key=key, issued_at=issued_at:
                                self._runtime_result(key, response.runtime, issued_at))
        if self.ctx.state in ('READY', 'RUNNING') and not self._chassis_ready():
            self._fault('C板状态超时，停止旧任务')
        state = self.ctx.state
        if state in ('INITIALIZING', 'RESETTING'):
            self._initialize()
        elif state in ('PAUSING', 'FAULT'):
            self._settle_stop()
        elif state == 'RUNNING':
            if not self._snapshots_ready():
                self._fault('执行端运行状态过期，停止推进')
                return
            if self._enter_pending:
                self._enter_pending = False
                self._enter_stage()
            elif self.ctx.substate == 'WAIT_MAP':
                self._commit_map()
            elif self.ctx.active and self.ctx.active.kind == 'route':
                self._verify_route()
            elif self.ctx.active and self.ctx.active.handle is not None:
                task = self.ctx.active
                runtime, stamp = self.runtime[task.executor]
                if stamp > task.started_at and not runtime.busy and runtime.last_task_id == task.task_id:
                    if not task.terminal_seen_at:
                        task.terminal_seen_at = now
                    elif now - task.terminal_seen_at > self.response_timeout:
                        self._fault('执行端已结束但Action结果未收到，未假定业务成功')
        if self.ctx.state == 'READY' and self._screen_start_at is not None:
            if time.monotonic() >= self._screen_start_at:
                result = self._start(Trigger.Request(), Trigger.Response())
                if not result.success:
                    self._fault('屏幕启动未完成：' + result.message)

    def _runtime_result(self, key, runtime, issued_at):
        if key in self.runtime and issued_at < self.runtime[key][1]:
            return
        if runtime.source != self.expected_source[key]:
            raise ValueError(f'{key}执行来源不符合{self.mode}模式')
        if key in self.reset_done and self.ctx.state in ('INITIALIZING', 'RESETTING') and runtime.run_id != self.ctx.run_id:
            return  # A get request issued just before reset may arrive after its acknowledgement.
        previous = self.sessions.get(key)
        self.runtime[key] = (runtime, issued_at)
        if previous and previous != runtime.executor_session_id:
            self.sessions[key] = runtime.executor_session_id
            raise ValueError(f'{key}执行节点中途重启，请人工恢复并复位')
        self.sessions[key] = runtime.executor_session_id
        if self.ctx.state not in ('WAIT_SETUP', 'INITIALIZING', 'RESETTING') and runtime.run_id != self.ctx.run_id:
            raise ValueError(f'{key}本轮身份丢失或冲突')
        if key == 'route' and self.ctx.state == 'RUNNING':
            self.ctx.current_heading = runtime.current_heading

    def _snapshots_ready(self):
        return len(self.runtime) == 3 and all(time.monotonic() - stamp < self.runtime_max_age for _, stamp in self.runtime.values())

    def _initialize(self):
        if time.monotonic() - self.init_started > self.init_timeout:
            self._fault('初始化超时：检查执行端、运动配置和未决动作')
            return
        if not self.task_clients.ready() or not self._snapshots_ready():
            return
        for key, (state, _) in list(self.runtime.items()):
            if key in self.reset_done:
                continue
            if state.busy or not state.quiescent or state.arm_pending:
                self._stop_executor(key, state)
                return
            tag = 'reset:' + key
            if tag in self.inflight:
                return
            if state.run_id == self.ctx.run_id:
                self.reset_done.add(key)
                continue
            reset_id = self.reset_ids.setdefault(key, str(uuid.uuid4()))
            request = ResetExecutor.Request(expected_run_id=state.run_id, run_id=self.ctx.run_id,
                                            reset_id=reset_id, home_node=self.home,
                                            initial_heading=self.initial_heading)
            self._track(tag, self.task_clients.reset[key].call_async(request),
                        lambda response, key=key: self._reset_result(key, response), mutation=True)
            return
        if len(self.reset_done) == 3:
            if not self._chassis_ready():
                raise ValueError('C板尚未就绪')
            route = self.runtime['route'][0]
            if not route.odometry_ready or not all(
                    runtime.run_id == self.ctx.run_id and runtime.config_ready and runtime.quiescent
                    and runtime.motion_released and runtime.travel_ready and not runtime.busy and not runtime.arm_pending
                    for runtime, _ in self.runtime.values()):
                return
            self.ctx.confirmed_node = self.home
            self.ctx.position_known = True
            self.ctx.current_heading = route.current_heading
            self.ctx.state = 'READY'
            self.ctx.substate = 'WAIT_SCREEN_START' if self._screen_start_at is not None else ''
            self.ctx.quiescent = True
            self.ctx.reason = ''
            next_step = '等待屏幕启动倒计时结束' if self._screen_start_at is not None else '人工恢复现场后调用start'
            self.get_logger().info(f'READY run={self.ctx.run_id} start={self.home} finish={self.finish_node} mode={self.mode} '
                                   f'sources={self.expected_source}；{next_step}')
            self._publish()

    def _reset_result(self, key, response):
        if response.runtime.source != self.expected_source[key] or response.runtime.executor_session_id != self.sessions[key]:
            raise ValueError('复位响应来自非预期执行端，检查同名服务冲突')
        if not response.accepted or response.runtime.run_id != self.ctx.run_id:
            raise ValueError(response.reason or '执行端拒绝复位')
        self.runtime[key] = (response.runtime, time.monotonic())
        self.reset_done.add(key)

    def _start(self, request, response):
        try:
            if self._screen_start_at is not None and time.monotonic() < self._screen_start_at:
                raise ValueError('屏幕 OK 后的 2 秒启动等待尚未结束')
            if self.ctx.state != 'READY' or not self.task_clients.ready() or not self._snapshots_ready():
                raise ValueError('需要READY且所有执行端在线；暂停/故障后先复位')
            for key, (state, _) in self.runtime.items():
                if (state.run_id != self.ctx.run_id or state.busy or not state.quiescent or state.arm_pending
                        or not state.config_ready or not state.motion_released or not state.travel_ready):
                    raise ValueError(f'{key}未就绪或配置未完成')
            route = self.runtime['route'][0]
            if not route.odometry_ready or not route.position_known or route.current_node != self.home:
                raise ValueError('路线起点或里程反馈未就绪')
            start(self.ctx)
            self._screen_start_at = None
            self._enter_pending = True
            response.success, response.message = True, '开始已接受，先斜移离开启停区'
            self.get_logger().info('比赛开始：先斜移离开启停区，再雷达扫描、前往二维码区')
        except ValueError as exc:
            response.message = str(exc)
        self._publish()
        return response

    def _configure_and_start(self, request, response):
        response.run_id = self.ctx.run_id
        try:
            if not self.screen_controlled or self.ctx.state != 'WAIT_SETUP':
                raise ValueError('仅屏幕控制模式的 WAIT_SETUP 状态接受本轮配置；重测先复位')
            if request.expected_run_id != self.ctx.run_id:
                raise ValueError('屏幕配置不属于当前等待轮次')
            home, finish, routes, edges = parse_screen_config(
                request.start_zone, request.stop_zone, request.forbidden_routes)
            self.home, self.finish_node = home, finish
            self.initial_heading = 'DOWN' if home == 1 else 'UP'
            self.ctx.home_node = home
            self.ctx.manual_forbidden_routes = routes
            self.ctx.manual_blocked_edges = edges
            self.ctx.state = 'INITIALIZING'
            self.ctx.substate = 'PREPARING_SCREEN_START'
            self._screen_start_at = time.monotonic() + 2.0
            self.init_started = time.monotonic()
            self.reset_done.clear()
            self.reset_ids.clear()
            response.accepted = True
            response.reason = '配置已接受；等待 2 秒且执行端就绪后开始比赛'
            self.get_logger().info(
                f'SCREEN_CONFIG start{request.start_zone}→节点{home} stop{request.stop_zone}→节点{finish} '
                f'manual_routes={routes} manual_edges={edges}')
        except ValueError as exc:
            response.reason = str(exc)
        self._publish()
        return response

    def _pause(self, request, response):
        try:
            pause(self.ctx)
            self._stop_control()
            self._stop_epoch = time.monotonic()
            if self.ctx.active and self.ctx.active.kind == 'route':
                self.ctx.position_known = False
            self._enter_pending = False
            response.success, response.message = True, '暂停已接受，等待当前机构动作安全结束'
            self.get_logger().info('PAUSING；停止推进，等待执行端停止')
        except ValueError as exc:
            response.message = str(exc)
        self._publish()
        return response

    def _reset(self, request, response):
        allowed = self.ctx.state in ('READY', 'PAUSED', 'FAULT', 'FINISHED')
        mutations = any(call['mutation'] for call in self.pending)
        stopped = self._snapshots_ready() and all(not state.busy and state.quiescent and state.motion_released and not state.arm_pending
                                                  for state, _ in self.runtime.values())
        if not allowed or not stopped or mutations:
            response.message = '先暂停并等待所有动作停止、未知请求明确后才能复位'
            return response
        self.ctx = MissionContext(home_node=self.home,
                                  state='WAIT_SETUP' if self.screen_controlled else 'RESETTING')
        self._screen_start_at = None
        self._stop_control()
        self._route_control_sequence = 0
        self._route_control_wait_started = None
        self.reset_done.clear()
        self.reset_ids.clear()
        self._stop_requested.clear()
        self._enter_pending = False
        self.init_started = time.monotonic()
        response.success, response.message = True, '软件复位已接受；请人工恢复车辆、物料及机构'
        if self.screen_controlled:
            response.message += '，然后在屏幕重新确认本轮配置'
        self._publish()
        return response

    def _fault(self, reason):
        self._screen_start_at = None
        self._stop_control()
        if self.ctx.state != 'FAULT':
            self.get_logger().error('FAULT ' + reason)
            self._stop_epoch = time.monotonic()
        self.ctx.state = 'FAULT'
        if self.ctx.active and self.ctx.active.kind == 'route':
            self.ctx.position_known = False
        self.ctx.reason = reason
        self.ctx.quiescent = False
        self._enter_pending = False

    def _stop_executor(self, key, state):
        tag = 'stop:' + key
        identity = (key, state.run_id, state.active_task_id)
        if tag in self.inflight or identity in self._stop_requested or not self.task_clients.stop[key].service_is_ready():
            return
        self._stop_requested.add(identity)
        request = StopExecutor.Request(run_id=state.run_id, task_id=state.active_task_id)
        self._track(tag, self.task_clients.stop[key].call_async(request),
                    lambda response, key=key, identity=identity: self._stop_result(key, identity, response),
                    mutation=True)

    def _stop_result(self, key, identity, response):
        if response.runtime.source != self.expected_source[key] or response.runtime.executor_session_id != self.sessions[key]:
            raise ValueError('停止响应来自非预期执行端，检查同名服务冲突')
        self.runtime[key] = (response.runtime, time.monotonic())
        if not response.accepted:
            self._stop_requested.discard(identity)
            self.get_logger().warning(key + '停止未接受：' + response.reason)

    def _settle_stop(self):
        task = self.ctx.active
        if task and task.handle is not None and 'cancel' not in self.inflight and not getattr(task, 'cancel_sent', False):
            task.cancel_sent = True
            self._track('cancel', task.handle.cancel_goal_async(), lambda _: None, mutation=True)
        if not self._snapshots_ready():
            return
        for key, (state, _) in self.runtime.items():
            if state.busy:
                self._stop_executor(key, state)
        self.ctx.arm_pending = any(state.arm_pending for state, _ in self.runtime.values())
        stopped = all(stamp >= self._stop_epoch and not state.busy and state.quiescent
                      and state.motion_released and not state.arm_pending
                      for state, stamp in self.runtime.values())
        mutations = any(call['mutation'] for call in self.pending)
        if stopped and not mutations:
            self.ctx.quiescent = True
            self.ctx.active = None
            if self.ctx.state == 'PAUSING':
                self.ctx.state = 'PAUSED'
                self.get_logger().info('PAUSED；当前动作结束，必须复位后重新开始')
                self._publish()

    def _enter_stage(self):
        c = self.ctx
        c.stage_started_at = time.monotonic()
        c.substate = 'ENTER'
        self.get_logger().info(f'ENTER stage={int(c.stage):02d} {c.stage.name} {LABELS[c.stage]}')
        try:
            if c.stage == Stage.READ_TASK_CODE:
                c.substate = 'WAIT_QR'
                self._accept_pending_qr()
            elif c.stage == Stage.STARTUP_SCAN:
                task = ActiveTask(str(uuid.uuid4()), 'scan', 'lidar')
                c.active = task
                goal = ScanRoads.Goal(run_id=c.run_id, task_id=task.task_id)
                self._send_action(self.task_clients.scan, goal, task)
            elif int(c.stage) in ROUTES or c.stage in (Stage.EXIT_START_AREA, Stage.ENTER_FINISH_AREA):
                maneuver = ''
                if c.stage in (Stage.EXIT_START_AREA, Stage.ENTER_FINISH_AREA):
                    maneuver = 'EXIT' if c.stage == Stage.EXIT_START_AREA else 'ENTER'
                    expected = goal_node = self.home if maneuver == 'EXIT' else self.finish_node
                else:
                    expected, goal_node = ROUTES[c.stage]
                    expected = self.home if expected is None else expected
                    goal_node = self.finish_node if goal_node is None else goal_node
                runtime = self.runtime['route'][0]
                if not runtime.position_known or runtime.current_node != expected or runtime.busy:
                    raise ValueError('路线起点未确认或与阶段预期冲突')
                task = ActiveTask(str(uuid.uuid4()), 'route', 'route', goal=goal_node)
                c.active = task
                c.position_known = False
                self._stop_control()
                self._route_control_sequence = 0
                self._route_control_wait_started = None
                req = PlanRoute.Request(run_id=c.run_id, request_id=task.task_id, stage=int(c.stage),
                                        start=0, goal=goal_node,
                                        align_at_goal=not maneuver and c.stage != Stage.RETURN_HOME,
                                        parking_maneuver=maneuver,
                                        parking_offset_mm=self.parking_offset_mm if maneuver else 0)
                c.substate = 'WAIT_ARRIVAL'
                self._track('plan', self.task_clients.route.call_async(req),
                            lambda response: self._plan_result(task, response), mutation=True)
            else:
                area, operation, batch, colors, positions = batch_spec(c.stage, c.code, c.first_layer)
                task = ActiveTask(str(uuid.uuid4()), 'work', 'work',
                                  spec=expected_steps(operation, colors, positions), batch=batch)
                c.active = task
                goal = RunWorkBatch.Goal(
                    run_id=c.run_id, task_id=task.task_id, task_revision=1,
                    stage=int(c.stage), batch=batch, operation=operation,
                    colors=list(colors), positions=list(positions), enable_motion=True,
                    first_layer=[FirstLayerRecord(color=color, position=position)
                                 for color, position in sorted(c.first_layer.items())] if operation == 'STORAGE_STACK' else [])
                self._send_action(self.task_clients.work[area], goal, task)
        except (ValueError, RuntimeError) as exc:
            self._fault(str(exc))
        self._publish()

    def _send_action(self, client, goal, task):
        if not client.server_is_ready():
            raise ValueError('Action执行端不可用')
        self.ctx.substate = 'WAIT_BATCH' if task.kind == 'work' else 'WAIT_SCAN'
        self._track('goal', client.send_goal_async(goal, feedback_callback=self._feedback),
                    lambda handle: self._goal_response(task, handle), mutation=True)

    def _goal_response(self, task, handle):
        if not handle.accepted:
            raise ValueError('执行端拒绝任务')
        task.accepted, task.handle = True, handle
        # Results may take arbitrarily long; executor heartbeat/runtime checks provide liveness.
        self._track('result', handle.get_result_async(), lambda response: self._action_result(task, response),
                    timeout=False, mutation=True)

    def _feedback(self, message):
        f = message.feedback
        c = self.ctx
        if not c.matches(f.run_id, f.task_id) or c.active.terminal is not None:
            return
        if c.state == 'RUNNING':
            c.substate = f.phase
        if c.active.kind == 'work':
            c.arm_pending = f.arm_pending
            try:
                self._commit_records(c.active, f.confirmed_steps)
            except ValueError as exc:
                self._fault(str(exc))

    def _commit_records(self, task, records):
        for _, value in commit_steps(self.ctx, task, records).items():
            operation, index, color, position = value
            self.get_logger().info(f'STEP {operation} item={index} color={color} position={position} CONFIRMED')

    def _action_result(self, task, response):
        c, r = self.ctx, response.result
        if not c.matches(r.run_id, r.task_id):
            raise ValueError('Action结果身份不匹配')
        if r.source != self.expected_source[task.executor] or r.executor_session_id != self.sessions[task.executor]:
            raise ValueError('Action结果来源或执行进程身份冲突')
        if task.kind == 'work':
            if r.task_revision != 1:
                raise ValueError('任务快照版本不匹配')
            self._commit_records(task, r.confirmed_steps)
        task.terminal = r.outcome
        if c.state != 'RUNNING':
            return
        if response.status != GoalStatus.STATUS_SUCCEEDED or r.outcome != 'COMPLETED' or not r.travel_ready:
            raise ValueError(r.reason or '整批/扫描未完整成功')
        if task.kind == 'scan':
            if not r.map_revision:
                raise ValueError('扫描地图版本无效')
            radar_edges = Topology().normalize_blocked((e.a, e.b) for e in r.blocked_edges)
            c.blocked_edges = tuple(sorted(radar_edges | set(c.manual_blocked_edges)))
            self.get_logger().info(
                f'MAP_MERGED manual_routes={c.manual_forbidden_routes} '
                f'radar_edges={sorted(radar_edges)} final_edges={c.blocked_edges}')
            c.map_revision = r.map_revision
            c.substate = 'WAIT_MAP'
            self._map_task = task.task_id
            self._map_started_at = time.monotonic()
            self._commit_map()
        else:
            ids = {record.step_id for record in r.confirmed_steps}
            if ids != {row[0] for row in task.spec} or not all((r.all_items_completed, r.reference_restored,
                                                             r.motion_released, r.arm_idle)):
                raise ValueError('整批动作、离站参考或控制权尚未完整确认')
            if c.stage == Stage.STORAGE_PLACE_1:
                mapping = {row.color: row.position for row in r.first_layer_records}
                if len(r.first_layer_records) != 3 or mapping != c.code.first_layer:
                    raise ValueError('首批暂存确认映射无效')
                c.first_layer = mapping
            c.arm_pending = False
            self._finish()

    def _commit_map(self):
        c = self.ctx
        if time.monotonic() >= self.next_map:
            self.next_map = time.monotonic() + 0.3
            self.map_pub.publish(BlockedEdges(run_id=c.run_id, map_revision=c.map_revision,
                                              edges=[Edge(a=a, b=b) for a, b in c.blocked_edges]))
        runtime = self.runtime['route'][0]
        content = tuple(sorted((min(e.a, e.b), max(e.a, e.b)) for e in runtime.blocked_edges))
        if runtime.run_id == c.run_id and runtime.map_ready and runtime.map_revision == c.map_revision and content == c.blocked_edges:
            self.get_logger().info(f'MAP_COMMITTED revision={c.map_revision} blocked={c.blocked_edges}')
            self._finish()
        elif c.active and time.monotonic() - self._map_started_at > self.init_timeout:
            self._fault('扫描/地图提交超时')

    def _plan_result(self, task, response):
        plan = response.plan
        if not self.ctx.matches(plan.run_id, plan.request_id):
            raise ValueError('路线响应身份冲突')
        if plan.status != 'ok':
            raise ValueError('路线拒绝：' + plan.status + ' ' + plan.reason)
        if plan.goal != task.goal or plan.map_revision != self.ctx.map_revision:
            raise ValueError('路线目标或地图版本冲突')
        task.accepted = True
        self._route_control_wait_started = time.monotonic()
        feedback = self._route_control
        if (self.ctx.state == 'RUNNING' and task.terminal is None and feedback is not None
                and self._snapshots_ready() and self._route_control_fresh(feedback)
                and time.monotonic() - self._route_control_received <= self.control_timeout):
            self.control_pub.publish(feedback.command)

    def _on_route(self, result):
        c = self.ctx
        if not c.matches(result.run_id, result.request_id) or c.active.kind != 'route':
            return
        if result.source != self.expected_source['route'] or result.executor_session_id != self.sessions.get('route'):
            self._fault('路线执行结果来源冲突')
            return
        if result.state in ('COMPLETED', 'ABORTED', 'REJECTED'):
            c.active.terminal = result.state
            self._stop_control()
            if result.state in ('ABORTED', 'REJECTED') and c.state == 'RUNNING':
                self._fault('路线执行中止：' + result.reason)

    def _verify_route(self):
        c, task = self.ctx, self.ctx.active
        runtime, stamp = self.runtime['route']
        if not task.accepted or stamp < task.started_at:
            return
        state = task.terminal
        if runtime.last_task_id == task.task_id:
            state = runtime.last_state
        if state in ('ABORTED', 'REJECTED'):
            self._fault('路线执行中止：' + runtime.reason)
        elif state == 'COMPLETED' and not runtime.busy and runtime.motion_released:
            # The terminal topic can precede a fresh query snapshot. Wait until the
            # runtime confirms this task before checking its destination.
            if runtime.last_task_id != task.task_id or runtime.last_state != 'COMPLETED':
                return
            if not runtime.position_known or runtime.current_node != task.goal or not runtime.heading_known:
                self._fault('路线终点状态未确认')
                return
            c.confirmed_node, c.position_known = task.goal, True
            c.current_heading = runtime.current_heading
            self._finish()

    def _on_qr(self, msg):
        c = self.ctx
        # The scanner releases its camera after one result. Retain a result that
        # arrives before READ_TASK_CODE so entering that stage cannot lose it.
        if (c.code is not None or c.pending_code is not None
                or c.state not in ('WAIT_SETUP', 'INITIALIZING', 'RESETTING', 'READY', 'RUNNING')
                or c.stage not in (Stage.EXIT_START_AREA, Stage.STARTUP_SCAN,
                                    Stage.MOVE_TO_QR, Stage.READ_TASK_CODE)):
            return
        try:
            c.pending_code = parse_task_code(msg.data)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=3.0)
            return
        self._accept_pending_qr()

    def _accept_pending_qr(self):
        c = self.ctx
        if (c.state != 'RUNNING' or c.stage != Stage.READ_TASK_CODE
                or c.substate != 'WAIT_QR' or c.code is not None or c.pending_code is None):
            return
        c.code, c.pending_code = c.pending_code, None
        self.get_logger().info('TASK_CODE_ACCEPTED ' + c.code.raw)
        self.qr_result_pub.publish(String(data=c.code.raw))
        self._publish()
        self._finish()

    def _finish(self):
        c = self.ctx
        self._stop_control()
        if c.state == 'RUNNING':
            self.get_logger().info(f'DONE stage={int(c.stage):02d} {c.stage.name} '
                                   f'elapsed={time.monotonic() - c.stage_started_at:.2f}s')
        if advance(c):
            self._enter_pending = c.state == 'RUNNING'
            if c.state == 'FINISHED':
                self.get_logger().info(f'FINISHED confirmed_steps={len(c.confirmed_steps)}')
            self._publish()

    def _publish(self):
        c = self.ctx
        now = time.monotonic()
        end = c.finished_at or now
        state = MissionState(
            run_id=c.run_id, mode=self.mode, state=c.state, stage=int(c.stage), stage_name=c.stage.name,
            substate=c.substate, active_task_id=c.active.task_id if c.active else '',
            active_task_kind=c.active.kind if c.active else '', batch=WORK.get(c.stage, ('', '', 0))[2],
            confirmed_step_count=len(c.confirmed_steps), quiescent=c.quiescent, arm_pending=c.arm_pending,
            confirmed_node=c.confirmed_node, position_known=c.position_known,
            current_heading=c.current_heading, map_revision=c.map_revision, reason=c.reason,
            elapsed_s=end - c.started_at if c.started_at else 0.0,
            stage_elapsed_s=end - c.stage_started_at if c.stage_started_at else 0.0)
        self.state_pub.publish(state)
        info = MissionTaskInfo(run_id=c.run_id, valid=c.code is not None, task_revision=1 if c.code else 0)
        if c.code:
            info.raw_code = c.code.raw
            for field in ('batch1_colors', 'batch1_positions', 'batch2_colors', 'batch2_positions'):
                setattr(info, field, list(getattr(c.code, field)))
            info.planned_first_layer = [FirstLayerRecord(color=color, position=position)
                                       for color, position in sorted(c.code.first_layer.items())]
            info.confirmed_first_layer = [FirstLayerRecord(color=color, position=position)
                                         for color, position in sorted(c.first_layer.items())]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            if rclpy.ok():
                node._stop_control()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
