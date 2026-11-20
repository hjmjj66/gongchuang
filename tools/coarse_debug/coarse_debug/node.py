"""Standalone browser controller with real gantry motion in all three areas."""

import argparse
import math
from pathlib import Path
import queue
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from agv_interfaces.msg import ArmExecution, CBoardStatus, ControlCommand, OdometryFeedback
from agv_interfaces.srv import ArmCommand, JogArm
from agv_planning.link_recovery import communication_lost
from agv_perception.link_pause import resume_task
from agv_perception.arm_servo import ArmServoPublisher
from agv_perception.vision_gate import VisionGate
from agv_perception.coarse_control import GantryPose, Pose
from route_debug.web_server import WebServer

from .config import area_fields, fixed_parameters, get_value, parameter_groups, parse_mode, parse_task, read_config, save_parameters
from .task import DebugTask, RawDebugTask, StorageDebugTask, parse_execution
from .tracking import TrackingTask, parse_tracking_target
from .pose_calibration import ReferenceAdjustment, PoseCalibrationTracking
from .calibration import CalibrationFrames, TYPES as CALIBRATION_TYPES, save_calibration
from .vision import RingVision

TOOL = Path(__file__).resolve().parents[1]


class DebugNode(Node):
    def __init__(self, args, *, vision=None):
        super().__init__('coarse_debug_controller')
        self._vision_gate = VisionGate()
        self.path = Path(args.config).resolve()
        self.config = read_config(self.path)
        self.area, self.visit = 'coarse', 1
        self.mode = 'task'
        self.calibration_context = ('ring', uuid.uuid4().hex, time.monotonic())
        self.calibration_frames = CalibrationFrames()
        self.calibration_sample = None
        self.reference_ready = None
        self.reference_owner = ''
        self.calibration_paths = {}
        installed_config = Path(get_package_share_directory('agv_perception'))/'config'
        for area in ('raw', 'coarse', 'storage'):
            source = TOOL.parents[1]/f'ros2_ws/src/agv_perception/config/{area}.yaml'
            configured = Path(self.config['debug'].get(f'calibration_{area}_config', source)).resolve()
            self.calibration_paths[area] = [configured]
            installed = installed_config/f'{area}.yaml'
            if configured == source.resolve() and installed.is_file() and installed.resolve() != configured:
                self.calibration_paths[area].append(installed)
        self.task = None
        self.owner = ''
        self.heartbeat_at = 0.
        self.board = self.pose = None
        self.board_at = float('-inf')
        self.arm = None
        self.arm_at = float('-inf')
        self._submitted = set()
        self._arm_future = None
        self._abort_id = ''
        self._expected_arm_id = ''
        self._servo = ArmServoPublisher(self)
        self._arm_client = self.create_client(ArmCommand, '/C/arm_command')
        self._jog_client = self.create_client(JogArm, '/C/arm_jog')
        self.binding = None
        self.graph_ok = False
        self.graph_error = '等待ROS节点发现'
        self._reported_state = ''
        self._last_web = 0.
        self.pub = self.create_publisher(ControlCommand, '/C/controll', 1)
        self.subscriptions_ = [
            self.create_subscription(CBoardStatus, '/C/status', self._board, 1),
            self.create_subscription(OdometryFeedback, '/C/odometry', self._odom, 1),
            self.create_subscription(ArmExecution, '/C/arm_execution', self._arm, 1)]
        p, d = self.config['coarse'], self.config['debug']
        self.vision = vision if vision is not None else RingVision(
            p['vision']['ring_config'], d['preview_width'], self.get_logger(),
            self.config['storage']['vision']['material_config'])
        self.web = WebServer(args.host or d['host'], args.port if args.port is not None else d['port'],
                             TOOL/'web', preview=self.vision.preview, preview_frame=self._preview_frame)
        self.web.start()
        self.graph_timer = self.create_timer(.5, self._check_graph)
        self.timer = self.create_timer(.02, self._tick)
        self.get_logger().info(f'原料/粗加工/暂存调试：http://127.0.0.1:{self.web.http.server_port}；局域网访问 http://小车IP:{self.web.http.server_port}')
        self.get_logger().info(f'参数保存位置：{self.path}；作业流程可选1s模拟或真实取放，追踪模式按钮执行真实取放')

    @property
    def active(self):
        return self.task is not None and not self.task.finished

    @property
    def parameters(self):
        return self.config[self.area]

    @property
    def preview_kind(self):
        return 'material' if self.area == 'raw' or (self.area == 'storage' and self.visit == 2) else 'ring'

    def _preview_frame(self):
        if not hasattr(self.vision, 'packet'):
            return self.vision.preview(), None
        packet = self.vision.packet()
        if packet is None:
            return None, None
        obs, jpeg = packet
        category, selection_id, selected_at = self.calibration_context
        if self.mode != 'calibration':
            return jpeg, None
        if obs.kind != CALIBRATION_TYPES[category]['kind'] or obs.captured_at <= selected_at:
            return None, None
        metadata = self.calibration_frames.register(obs, category, selection_id)
        metadata['age_s'] = max(0., time.monotonic()-obs.captured_at)
        return jpeg, metadata

    def _sample_time(self, msg):
        stamp = msg.header.stamp
        age = (self.get_clock().now().nanoseconds-stamp.sec*1_000_000_000-stamp.nanosec)/1e9
        if 0 <= age <= self.parameters['execution']['feedback_timeout_s']:
            return time.monotonic()-age
        return None

    def _board(self, msg):
        at = self._sample_time(msg)
        if at is not None and math.isfinite(msg.yaw_rad):
            self.board, self.board_at = msg, at

    def _odom(self, msg):
        at = self._sample_time(msg)
        if (at is not None and msg.valid and msg.origin_initialized and
                all(math.isfinite(v) for v in (msg.x_m, msg.y_m, msg.yaw_rad, msg.vx_mps, msg.vy_mps))):
            self.pose = (Pose(msg.x_m, msg.y_m, msg.yaw_rad, at, msg.vx_mps, msg.vy_mps),
                         msg.odom_session_id, msg.motion_epoch)

    def _arm(self, message):
        at = self._sample_time(message)
        if at is not None:
            self.arm, self.arm_at = message, at

    def _arm_idle(self):
        return bool(self.arm and self.arm.idle and not self.arm.pending and self._arm_future is None and
                    (not self._expected_arm_id or self.arm.command_id == self._expected_arm_id))

    def _healthy(self):
        return not self._health_error()

    def _health_error(self):
        now = time.monotonic()
        limit = self.parameters['execution']['feedback_timeout_s']
        if not self.graph_ok:
            return self.graph_error
        if not self.board:
            return '未收到C板状态反馈'
        if now-self.board_at > limit:
            return f'C板状态反馈超时：{now-self.board_at:.2f}s，限值{limit:.2f}s'
        if not self.board.connected:
            return 'C板串口未连接'
        if not self.board.status_valid:
            return 'C板状态无效：'+self.board.fault_reason
        if not self.board.pc_control:
            return 'C板未处于PC控制模式'
        if self.board.error_flags:
            return f'C板故障：error_flags=0x{self.board.error_flags:x}；{self.board.fault_reason}'
        if not self.board.motion_allowed:
            return 'C板禁止运动：'+self.board.fault_reason
        if not self.pose:
            return '未收到有效且已初始化的里程反馈'
        if now-self.pose[0].received_at > limit:
            return f'里程反馈超时：{now-self.pose[0].received_at:.2f}s，限值{limit:.2f}s'
        if self.pose[1:] != (self.board.odom_session_id, self.board.motion_epoch):
            return '里程与C板坐标会话不一致（odom_session_id / motion_epoch）'
        if not self.arm or now-self.arm_at > limit:
            return '机构执行状态反馈缺失或超时'
        if self.arm.state == 'UNKNOWN' or self.board.arm_state == 3:
            return '机构执行状态异常：'+self.arm.reason
        if not self._arm_client.service_is_ready():
            return '机构指令服务 /C/arm_command 不可用'
        if self.area == 'coarse' and not self._jog_client.service_is_ready():
            return '机构点动服务 /C/arm_jog 不可用'
        return ''

    def _check_graph(self):
        errors = []
        for topic in ('/C/controll', '/C/odometry', '/C/status'):
            writers = self.get_publishers_info_by_topic(topic)
            if len(writers) != 1 or (topic == '/C/controll' and writers[0].node_name != self.get_name()):
                names = ', '.join(f'{w.node_namespace.rstrip("/")}/{w.node_name}' for w in writers)
                errors.append(f'{topic} 发布者应为1个，实际{len(writers)}个（{names or "无"}）')
        conflicts = [name for name, _ in self.get_node_names_and_namespaces()
                     if name in ('mission_node', 'route_debug_controller', 'coarse_node')]
        if conflicts:
            errors.append('其他控制节点正在运行：'+', '.join(conflicts))
        self.graph_error = 'ROS独占控制检查失败：'+'；'.join(errors) if errors else ''
        self.graph_ok = not errors

    def _can_start(self, observation):
        return bool(self._arm_idle() and self._healthy() and self.vision.ready and not self.vision.error and
                    not (hasattr(self.vision,'material_model_state') and
                         self.vision.material_model_state()['switching']) and
                    observation and observation.kind == self.preview_kind and
                    0 <= time.monotonic()-observation.captured_at <=
                    self.parameters['vision']['max_frame_age_s'])

    def _command(self, action, settings, client):
        if action == 'heartbeat':
            if client == self.owner:
                self.heartbeat_at = time.monotonic()
        elif action == 'stop':
            self._stop('网页停止', failed=False)
        elif action == 'material_model':
            if self.active:
                raise ValueError('任务正在运行，请先停止再切换物料模型')
            if self._arm_future is not None:
                raise ValueError('机构指令仍在等待响应，请稍后再切换')
            if not hasattr(self.vision,'request_material_model'):
                raise ValueError('当前识别源不支持切换物料模型')
            self.vision.request_material_model(settings.get('use_jhb_model', False),
                                               test_combine=settings.get('test_combine', False))
            category = self.calibration_context[0]
            self.calibration_context = (category,uuid.uuid4().hex,time.monotonic())
            self.calibration_frames = CalibrationFrames()
            self.calibration_sample = None
            self.reference_ready = None
        elif action == 'select':
            area, visit = parse_mode(settings)
            mode = settings.get('mode', self.mode)
            if mode not in ('task', 'tracking', 'calibration'):
                raise ValueError('模式须为作业流程、追踪模式或标定模式')
            if self.active:
                raise ValueError('任务正在运行，请先停止再切换模式、区域或次数')
            if self._arm_future is not None:
                raise ValueError('机构指令仍在等待响应，请稍后再切换')
            # Selection only changes the preview/configuration. Hardware readiness
            # belongs to _can_start(), so missing feedback must not block selection.
            if mode == 'calibration':
                category = settings.get('calibration_type', 'raw' if area == 'raw' else 'stack' if area == 'storage' and visit == 2 else 'ring')
                if category not in CALIBRATION_TYPES:
                    raise ValueError('请选择有效的标定类型')
                spec = CALIBRATION_TYPES[category]
                area, visit = spec['area'], spec['visit']
                self.calibration_context = (category, uuid.uuid4().hex, time.monotonic())
                self.calibration_sample = None
                self.reference_ready = None
            self.area, self.visit, self.mode = area, visit, mode
            self.task = None
            self.vision.request(self.preview_kind, largest_per_color=self.area == 'raw')
        elif action == 'start':
            if self.mode == 'calibration':
                raise ValueError('标定模式仅预览和采点，不启动运动')
            area, visit = parse_mode(settings)
            mode = settings.get('mode', 'task')
            if mode != self.mode:
                raise ValueError('模式已变化，请刷新后重新开始')
            if mode == 'tracking':
                target = parse_tracking_target(settings, self.preview_kind)
            else:
                area, visit, order, first_layer = parse_task(settings)
                execution = parse_execution(settings)
            if self.active:
                raise ValueError('任务正在运行，请先停止')
            if (area, visit) != (self.area, self.visit):
                raise ValueError('区域或次数已变化，请刷新后重新开始')
            if not self._can_start(self.vision.latest()):
                raise ValueError('尚未就绪：检查独占控制、PC 模式、里程和新鲜相机画面')
            self._begin_motion(client)
            if mode == 'tracking':
                self.task = TrackingTask(self.parameters, uuid.uuid4().hex, target,
                                         self.preview_kind, self.heartbeat_at, area, visit)
            elif area == 'coarse':
                self.task = DebugTask(self.parameters, uuid.uuid4().hex, order, self.heartbeat_at,
                                      execution=execution)
            elif area == 'raw':
                self.task = RawDebugTask(self.parameters, uuid.uuid4().hex, order, self.heartbeat_at,
                                        execution=execution)
            else:
                self.task = StorageDebugTask(self.parameters, uuid.uuid4().hex, order,
                                            self.heartbeat_at, visit, first_layer,
                                            execution=execution)
        elif action == 'calibration_track_fb':
            if self.mode != 'calibration' or self.preview_kind != 'ring':
                raise ValueError('请先选择圆环相关的标定类型')
            if self.active:
                raise ValueError('请先停止当前运动')
            if not self._can_start(self.vision.latest()):
                raise ValueError('尚未就绪：检查独占控制、PC模式、里程、机构空闲和新鲜画面')
            self.reference_ready = None
            self._begin_motion(client)
            self.task = TrackingTask(self.parameters, uuid.uuid4().hex, 2, 'ring',
                                     self.heartbeat_at, self.area, fb_only=True)
        elif action in ('preset_adjust', 'preset_track'):
            if (hasattr(self.vision, 'material_model_state') and
                    self.vision.material_model_state()['switching']):
                raise ValueError('正在切换物料模型，请稍候再开始')
            category = self.calibration_context[0]
            if self.mode != 'calibration' or not CALIBRATION_TYPES[category].get('preset'):
                raise ValueError('请先选择粗加工 / 暂存区共用机构初值标定')
            restarting = action == 'preset_track' and self.active and isinstance(self.task, PoseCalibrationTracking)
            if self.active and not restarting:
                raise ValueError('请先停止当前运动再调整或切换目标')
            error = self._health_error()
            if error:
                raise ValueError(error)
            if not restarting and not self._arm_idle():
                raise ValueError('机构尚未空闲或指令仍在等待响应')
            if action == 'preset_adjust':
                self.reference_ready = None
                task = ReferenceAdjustment(self.parameters, uuid.uuid4().hex, time.monotonic())
            else:
                target = settings.get('target')
                if type(target) is not int or target not in (1, 3):
                    raise ValueError('请选择1号或3号圆环')
                task = PoseCalibrationTracking(self.parameters, uuid.uuid4().hex, target, time.monotonic(), self.area)
            if restarting:
                self.task.stop(time.monotonic(), '重新开始追踪', failed=False)
                self._servo.publish(None, self.binding)
            self._begin_motion(client)
            self.task = task
        elif action == 'preset_save':
            if self.mode != 'calibration' or not CALIBRATION_TYPES[self.calibration_context[0]].get('preset'):
                raise ValueError('请先选择粗加工 / 暂存区共用机构初值标定')
            target = settings.get('target', self.task.target if isinstance(self.task, PoseCalibrationTracking) else None)
            if type(target) is not int or target not in (1, 3):
                raise ValueError('请选择1号或3号圆环')
            now = time.monotonic()
            gantry = GantryPose.from_status(self.board, self.board_at) if self.board else None
            if gantry is None or now-self.board_at > self.parameters['execution']['feedback_timeout_s']:
                raise ValueError('当前机构位置反馈无效或过期，无法保存实际伸缩脉冲和转角')
            self._stop('保存当前机构位置', failed=False)
            self.task = PoseCalibrationTracking(self.parameters, uuid.uuid4().hex, target, now, self.area)
            self.task.request_save(gantry)
            self.owner = client
            self.config = save_calibration(self.path, self.config, self.calibration_context[0],
                self.task.sample, self.calibration_paths, ring=target)
            self.task.saved, self.task.state = True, 'DONE'
        elif action == 'tracking_action':
            if self.mode != 'tracking':
                raise ValueError('标定模式不执行取放动作')
            if not self.active or not isinstance(self.task, TrackingTask):
                raise ValueError('请先开始追踪，再执行取放动作')
            if client != self.owner:
                raise ValueError('请在启动本次追踪的页面执行取放动作')
            self.task.request_action(time.monotonic())
        elif action == 'calibration_capture':
            if self.mode != 'calibration' or self.active:
                raise ValueError('请先切换到标定模式')
            if CALIBRATION_TYPES[self.calibration_context[0]].get('preset'):
                raise ValueError('机构初值请使用追踪和保存机构初值按钮')
            if self.vision.error or not self.vision.ready:
                raise ValueError('相机或识别尚未就绪')
            category, selection_id, _ = self.calibration_context
            self.calibration_sample = dict(self.calibration_frames.capture(settings, category, selection_id,
                time.monotonic(), self.parameters['vision']['max_frame_age_s']), client=client, saved=False)
        elif action == 'calibration_save':
            sample = self.calibration_sample
            if (self.mode != 'calibration' or self.active or not sample or sample['client'] != client or
                    sample['id'] != settings.get('sample_id') or sample['category'] != self.calibration_context[0]):
                raise ValueError('请先在当前页面采集当前类型的标定点')
            self.config = save_calibration(self.path, self.config, sample['category'], sample['point'], self.calibration_paths)
            sample['saved'] = True
            self.get_logger().info(f'标定点已同步网页与正式比赛：{sample["category"]} {sample["point"]}')
        elif action == 'save':
            if self.active:
                raise ValueError('请先停止任务再保存参数')
            self.config = save_parameters(self.path, self.config, settings, self.area, self.calibration_paths)
            self.get_logger().info('参数已保存并同步正式比赛配置，下一次启动任务生效')
        else:
            raise ValueError('未知操作')

    def _begin_motion(self, client):
        self._link_paused_at = None
        self.binding = (self.board.odom_session_id, self.board.motion_epoch)
        self.owner, self.heartbeat_at = client, time.monotonic()
        self._reported_state = ''
        self._submitted.clear()
        self._abort_id = ''

    def _stop(self, reason, failed=True):
        was_active = self.active
        self.reference_ready = None
        if was_active:
            self.task.stop(time.monotonic(), reason, failed)
        if self.mode != 'calibration' or was_active:
            self._publish_velocity((0., 0., 0.))
        if self.binding:
            self._servo.publish(None, self.binding)
        if was_active and not self._abort_id and self._arm_client.service_is_ready():
            self._abort_arm()

    def _abort_arm(self):
        self._abort_id = uuid.uuid4().hex
        self._expected_arm_id = self._abort_id
        self._arm_future = (self._arm_client.call_async(ArmCommand.Request(
            command_id=self._abort_id, cmd=0)), time.monotonic())

    def _recover_connection(self, now):
        if self.active or self.binding is None or self.board is None or self._arm_future is not None:
            return
        binding = (self.board.odom_session_id, self.board.motion_epoch)
        if binding == self.binding:
            return
        limit = self.parameters['execution']['feedback_timeout_s']
        if (not self.graph_ok or now-self.board_at > limit or
                not self.board.connected or not self.board.status_valid or
                not self.board.pc_control or not self.board.motion_allowed or self.board.error_flags or
                not self.arm or now-self.arm_at > limit or not self._arm_client.service_is_ready()):
            return
        # An old stop may have been rejected while disconnected. Confirm a new
        # stop in the current session before accepting another manual start.
        self.binding = binding
        self.reference_ready = None
        self._abort_arm()

    def _publish_velocity(self, velocity):
        vx, vy, wz = velocity
        binding = self.binding or ((self.board.odom_session_id, self.board.motion_epoch) if self.board else ('', 0))
        self.pub.publish(ControlCommand(vx_mps=float(vx), vy_mps=float(vy), omega_deg_s=math.degrees(wz),
                                        odom_session_id=binding[0], motion_epoch=binding[1]))

    def _tick(self):
        self._check_arm_future()
        for _ in range(64):
            try:
                action, settings, client, key = self.web.commands.get_nowait()
            except queue.Empty:
                break
            try:
                self._command(action, settings, client)
            except (ValueError, TypeError, OSError) as error:
                self.web.finish(key, str(error))
            else:
                self.web.finish(key)
        obs = self.vision.latest()
        now = time.monotonic()
        if self.active:
            try:
                paused = getattr(self, '_link_paused_at', None)
                recovering = (self.board is not None and
                              self.binding == (self.board.odom_session_id, self.board.motion_epoch) and
                              (communication_lost(self.board) or
                               (paused is not None and not self.board.fault_reason and self._health_error())))
                if recovering and now-self.heartbeat_at <= self.config['debug']['browser_timeout_s']:
                    if paused is None:
                        self._link_paused_at = now
                    self._publish_velocity((0., 0., 0.))
                    self._servo.publish(None, self.binding)
                    self._publish_web(obs, now)
                    return
                if paused is not None:
                    resume_task(self.task, now-paused)
                    self._link_paused_at = None
                error = self._health_error()
                if error:
                    raise RuntimeError(error)
                if self.binding != (self.board.odom_session_id, self.board.motion_epoch):
                    raise RuntimeError('运行中坐标会话变化（odom_session_id / motion_epoch），请重新开始')
                if now-self.heartbeat_at > self.config['debug']['browser_timeout_s']:
                    raise RuntimeError('控制页面失联，任务已停止')
                if self.vision.error:
                    raise RuntimeError(self.vision.error)
                if isinstance(self.task, TrackingTask):
                    self.task.tick(now, self.pose[0], obs, self.arm, arm_at=self.arm_at,
                                   gantry=GantryPose.from_status(self.board, self.board_at))
                else:
                    self.vision.request(self.task.vision_kind, geometry_target=self.task.vision_target,
                                        largest_per_color=self.area == 'raw')
                    observation = self.vision.latest()
                    now = time.monotonic()
                    observation = self._vision_gate.filter(self.task, observation, now, self.area)
                    if not self.task.vision_kind or observation is not None:
                        self.task.tick(now, self.pose[0], observation, self.arm,
                                       GantryPose.from_status(self.board, self.board_at))
                self._servo.publish(self.task, self.binding, hold_zero=self.area == 'storage' and
                                    self.task.state in ('SCAN_FIRST_LAYER','ALIGN_RING','ALIGN_MATERIAL'))
                self._submit_arm()
                if isinstance(self.task, ReferenceAdjustment) and self.task.state == 'DONE':
                    self.reference_ready, self.reference_owner = (self.area, self.binding), self.owner
            except (ValueError, RuntimeError, OSError) as error:
                self._stop(str(error))
        self._recover_connection(now)
        self.vision.request(self.preview_kind, geometry_target=self.task.vision_target if self.active else 0,
                            largest_per_color=self.area == 'raw')
        if self.mode != 'calibration' or isinstance(self.task, (ReferenceAdjustment, TrackingTask)):
            self._publish_velocity(self.task.velocity if self.active else (0., 0., 0.))
        if self.task and self.task.state != self._reported_state:
            self._reported_state = self.task.state
            self.get_logger().info(f'{self.task.state} item={min(self.task.item,2)+1} reference_xy={self.task.reference} '
                                   f'positions={self.task.placements} reason={self.task.reason}')
        if now-self._last_web >= .1:
            self._last_web = now
            self._publish_web(obs, now)

    def _submit_arm(self):
        request = self.task.arm_request
        if request is None or request['command_id'] in self._submitted:
            return
        if (self.area == 'coarse' or isinstance(self.task, TrackingTask)) and not self._arm_idle():
            return
        self._publish_velocity((0., 0., 0.))
        self._expected_arm_id = request['command_id']
        self._submitted.add(request['command_id'])
        binding = dict(odom_session_id=self.binding[0], motion_epoch=self.binding[1])
        if request['cmd'] is None:
            future = self._jog_client.call_async(JogArm.Request(command_id=request['command_id'], **binding,
                       ud_rpm=-self.parameters['arm']['lower_speed_rpm'],
                       duration_s=self.parameters['arm']['lower_duration_s']))
        else:
            future = self._arm_client.call_async(ArmCommand.Request(**request, **binding))
        self._arm_future = (future, time.monotonic())

    def _check_arm_future(self):
        if self._arm_future is None:
            return
        future, at = self._arm_future
        if not future.done() and time.monotonic()-at <= self.parameters['execution']['arm_request_timeout_s']:
            return
        self._arm_future = None
        try:
            if not future.done():
                raise RuntimeError('机构请求结果未知，禁止重发')
            response = future.result()
            if not response.accepted:
                raise RuntimeError('机构请求被拒绝：'+response.reason)
        except Exception as error:
            self._stop(str(error))

    def _publish_web(self, obs, now):
        task = self.task
        pose = self.pose[0] if self.pose else None
        with self.web.lock:
            operations = [dict(client=k[0], **v[1]) for k, v in list(self.web.operations.items())[-20:]]
        self.web.publish(dict(
            area=self.area, visit=self.visit, mode=self.mode, preview_kind=self.preview_kind,
            task_execution=getattr(task, 'execution', 'simulated'),
            calibration_type=self.calibration_context[0], calibration_selection_id=self.calibration_context[1],
            calibration_sample=self.calibration_sample,
            calibration_points={name: self.config[spec['destinations'][0][0]]['vision'][spec['destinations'][0][1]]
                                for name, spec in CALIBRATION_TYPES.items() if not spec.get('preset')},
            gantry_presets={area: self.config[area].get('gantry_presets', {}).get('positions', {})
                            for area in ('coarse', 'storage')},
            preset_reference_ready=self.reference_ready == (self.area, self.binding) and self.reference_owner == self.owner,
            preset_saved=isinstance(task, PoseCalibrationTracking) and task.saved,
            tracking_target=task.target if isinstance(task, TrackingTask) else None,
            calibration_fb_tracking=self.mode == 'calibration' and isinstance(task, TrackingTask) and task.fb_only,
            state=task.state if task else 'IDLE', active=self.active, owner=self.owner,
            can_start=self.mode != 'calibration' and not self.active and self._can_start(obs), healthy=self._healthy(), graph_ok=self.graph_ok,
            health_error=self._health_error(), graph_error=self.graph_error,
            preset_can_start=not self.active and self._can_start(obs),
            reason=task.reason if task else '', vision_error=self.vision.error,
            order=(task.colors if self.area == 'raw' else task.positions) if task else (),
            item=min(task.item, 2)+1 if task else None,
            colors=task.colors if task else (),
            first_layer=task.first_layer if isinstance(task, StorageDebugTask) else {},
            seen_colors=sorted(task.seen_colors) if isinstance(task, StorageDebugTask) else [],
            target_ring=task.positions[min(task.item,len(task.positions)-1)] if task and not task.finished and self.area != 'raw' else None,
            tracked_color=task.tracked_color if isinstance(task, RawDebugTask) and self.active else None,
            pose=dict(x=pose.x, y=pose.y, yaw=pose.yaw, vx=pose.vx, vy=pose.vy) if pose else None,
            reference=task.reference if task else None, placements=task.placements if task else {},
            confirmed=task.confirmed if task else [], velocity=task.velocity if self.active else (0.,0.,0.),
            entry_alignment=task.entry_alignment if task else 'NOT_RUN',
            final_alignment=task.final_alignment if task else 'NOT_RUN',
            diagnostic=task.diagnostic if task else {},
            vision_performance=self.vision.metrics() if hasattr(self.vision, 'metrics') else {},
            material_model=self.vision.material_model_state() if hasattr(self.vision,'material_model_state') else None,
            base_locked=task.base_locked if isinstance(task, (RawDebugTask, StorageDebugTask)) else False,
            arm_request=task.arm_request if task else None,
            arm_velocity=task.arm_velocity if task is not None else (0., 0.),
            arm=dict(state=self.arm.state, pending=self.arm.pending, idle=self.arm.idle,
                     command_id=self.arm.command_id) if self.arm else None,
            arm_position=dict(fb_pulses=self.board.fb_pulses, ud_pulses=self.board.ud_pulses,
                              yaw_rad=self.board.yaw_rad, pos_flags=self.board.pos_flags) if self.board else None,
            vision=dict(kind=obs.kind, sequence=obs.sequence, age_s=now-obs.captured_at, image_size=obs.image_size,
                        rings=obs.rings, materials=obs.materials, center_sources=obs.center_sources) if obs else None,
            production_paths=[str(p) for p in self.calibration_paths[self.area]],
            config_path=str(self.path), fields=area_fields(self.area),
            fixed_parameters=fixed_parameters(self.config, self.area),
            motion_config_path=self.config.get('motion_config'), parameter_groups=parameter_groups(self.area),
            browser_timeout_s=self.config['debug']['browser_timeout_s'],
            parameters={item['path']: get_value(self.config, item['path'], self.area)
                        for item in area_fields(self.area)},
            operations=operations))

    def close(self):
        self._stop('工具退出', failed=False)
        self.timer.cancel()
        self.vision.close()
        self.web.close()


def preflight():
    rclpy.init()
    node = rclpy.create_node('coarse_debug_preflight')
    try:
        until = time.monotonic()+1.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        conflicts = [t for t in ('/C/controll', '/C/odometry', '/C/status') if node.get_publishers_info_by_topic(t)]
        conflicts.extend(name for name, _ in node.get_node_names_and_namespaces()
                         if name in ('mission_node', 'coarse_node', 'route_debug_controller', 'coarse_debug_controller', 'cboard_bridge_node'))
        if conflicts:
            raise SystemExit('请先退出已有控制与桥接进程：'+', '.join(sorted(set(conflicts))))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='三区域底盘与龙门架网页调试')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--config', default=str(TOOL/'config/debug.yaml'))
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
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
                node.vision.close()
                node.web.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
