"""Shared REAL raw/coarse/storage executor; mission owns the final base commands."""

import math
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future

from agv_interfaces.action import RunWorkBatch
from agv_interfaces.msg import ArmExecution, CBoardStatus, ControlCommand, ExecutorRuntime, FirstLayerRecord, OdometryFeedback, RouteControl, WorkStepRecord
from agv_interfaces.srv import ArmCommand, JogArm
from agv_planning.executor_contract import ExecutorContract

from agv_planning.link_recovery import communication_lost
from .link_pause import resume_task
from .arm_servo import ArmServoPublisher
from .coarse_control import CoarseTask, GantryPose, Pose, read_coarse_config, validate_coarse_config
from .coarse_vision import CoarseVision
from .vision_gate import VisionGate
from .raw_control import RawTask, read_raw_config, validate_raw_config
from .storage_control import StorageTask, read_storage_config, validate_storage_config


class CoarseNode(Node):
    def __init__(self, *, vision=None, **kwargs):
        super().__init__('coarse_node', **kwargs)
        default = str(Path(get_package_share_directory('agv_perception'))/'config/coarse.yaml')
        self.p = read_coarse_config(self.declare_parameter('config', default).value)
        storage_default = str(Path(get_package_share_directory('agv_perception'))/'config/storage.yaml')
        storage = read_storage_config(self.declare_parameter('storage_config', storage_default).value)
        raw_default = str(Path(get_package_share_directory('agv_perception'))/'config/raw.yaml')
        raw = read_raw_config(self.declare_parameter('raw_config', raw_default).value)
        self.configs = {'COARSE_PROCESS': self.p, 'RAW_PICK': raw,
                        'STORAGE_PLACE': storage, 'STORAGE_STACK': storage}
        self.area = 'coarse'
        self.config_errors = {}
        for operation, config in self.configs.items():
            try:
                validate = {'COARSE_PROCESS': validate_coarse_config, 'RAW_PICK': validate_raw_config,
                            'STORAGE_PLACE': validate_storage_config, 'STORAGE_STACK': validate_storage_config}[operation]
                validate(config)
                if operation == 'RAW_PICK':
                    if config['execution']['lease_timeout_s'] != self.p['execution']['lease_timeout_s']:
                        raise ValueError('共享执行端 lease_timeout_s 须一致')
                    continue
                for section, keys in {'vision': ('ring_config', 'material_config', 'model_runtime_path',
                                                 'max_frame_age_s', 'camera_start_timeout_s'),
                                      'execution': ('control_rate_hz', 'lease_timeout_s')}.items():
                    if any(config[section][key] != self.p[section][key] for key in keys):
                        raise ValueError(f'共享执行端配置须一致：{section}.{keys}')
            except (ValueError, KeyError, TypeError) as error:
                self.config_errors[operation] = f'{operation}: {error}'
                self.get_logger().warning(self.config_errors[operation])
        self.vision = vision if vision is not None else CoarseVision(
            self.p['vision'], self.get_logger(), raw_config=raw['vision'])
        self._vision_gate = VisionGate()
        self.task = self.goal = self._done = None
        self._reserved = False
        self.board = self.odom = self.arm = None
        self._board_at = self._odom_at = self._arm_at = float('-inf')
        self._binding = None
        self.last_task_id = ''
        self._stowed = True
        self._sequence = 0
        self._submitted = set()
        self._arm_future = None
        self._abort_id = ''
        self._expected_arm_id = ''
        self._reported_state = ''
        self._last_diagnostic = 0.
        self._control_pubs = {area: self.create_publisher(RouteControl, f'/vision/{area}/control', 10)
                              for area in ('raw', 'coarse', 'storage')}
        self._servo = ArmServoPublisher(self)
        self._arm_client = self.create_client(ArmCommand, '/C/arm_command')
        self._jog_client = self.create_client(JogArm, '/C/arm_jog')
        self._subs = [
            self.create_subscription(CBoardStatus, '/C/status', self._on_board, 1),
            self.create_subscription(OdometryFeedback, '/C/odometry', self._on_odom, 1),
            self.create_subscription(ArmExecution, '/C/arm_execution', self._on_arm, 1),
        ]
        self.contract = ExecutorContract(self, '/vision', 'REAL', self.p['execution']['lease_timeout_s'],
                                         self._runtime, self._stop, self._reset)
        self._action = ActionServer(self, RunWorkBatch, '/vision/coarse/run_batch', self._execute,
                                    goal_callback=self._accept, cancel_callback=self._cancel)
        self._storage_action = ActionServer(self, RunWorkBatch, '/vision/storage/run_batch', self._execute,
                                            goal_callback=self._accept_storage, cancel_callback=self._cancel)
        self._raw_action = ActionServer(self, RunWorkBatch, '/vision/raw/run_batch', self._execute,
                                        goal_callback=self._accept_raw, cancel_callback=self._cancel)
        self._timer = self.create_timer(1/max(p['execution']['control_rate_hz'] for p in self.configs.values()), self._tick)

    def _sample_time(self, message):
        stamp = message.header.stamp
        age = (self.get_clock().now().nanoseconds-stamp.sec*1_000_000_000-stamp.nanosec)/1e9
        if not 0 <= age <= self.p['execution']['feedback_timeout_s']:
            return None
        return time.monotonic()-age

    def _on_board(self, message):
        at = self._sample_time(message)
        if at is not None and math.isfinite(message.yaw_rad):
            self.board, self._board_at = message, at

    def _on_odom(self, message):
        at = self._sample_time(message)
        if at is not None and all(math.isfinite(v) for v in (message.x_m, message.y_m, message.yaw_rad, message.vx_mps, message.vy_mps)):
            self.odom, self._odom_at = message, at

    def _on_arm(self, message):
        at = self._sample_time(message)
        if at is not None:
            self.arm, self._arm_at = message, at

    def _healthy(self):
        now = time.monotonic()
        limit = self.p['execution']['feedback_timeout_s']
        return bool(self.board and self.odom and self.arm and
                    all(now-at <= limit for at in (self._board_at, self._odom_at, self._arm_at)) and
                    self.board.motion_allowed and self.board.pc_control and self.board.status_valid and
                    self.board.error_flags == 0 and self.board.arm_state != 3 and self.arm.state != 'UNKNOWN' and
                    self.odom.valid and self.odom.origin_initialized and
                    (self.odom.odom_session_id, self.odom.motion_epoch) ==
                    (self.board.odom_session_id, self.board.motion_epoch))

    def _config_ready(self, operation=None):
        error = bool(self.config_errors) if operation is None else operation in self.config_errors
        return bool(not error and self.vision.ready and not self.vision.error and
                    self._arm_client.service_is_ready() and
                    (operation != 'COARSE_PROCESS' or self._jog_client.service_is_ready()))

    def _idle(self):
        return bool(self.arm and time.monotonic()-self._arm_at <= self.p['execution']['feedback_timeout_s']
                    and (not self._expected_arm_id or self.arm.command_id == self._expected_arm_id)
                    and self._arm_future is None
                    and self.arm.idle and not self.arm.pending)

    def _runtime(self):
        busy = self._reserved or self.goal is not None
        pending = (bool(self.arm and self.arm.pending) or self._arm_future is not None or
                   bool(self._expected_arm_id and (not self.arm or self.arm.command_id != self._expected_arm_id)))
        return ExecutorRuntime(busy=busy, quiescent=not busy and not pending and self._idle(),
                               motion_released=not busy, arm_pending=pending,
                               travel_ready=not busy and self._stowed and self._idle(),
                               config_ready=self._config_ready(),
                               active_task_id=self.goal.request.task_id if self.goal else '',
                               last_task_id=self.last_task_id,
                               last_state=self.task.state if self.task else 'IDLE',
                               reason='; '.join(self.config_errors.values()) or self.vision.error or (self.task.reason if self.task else ''),
                               odometry_ready=self._healthy(),
                               odom_session_id=self.board.odom_session_id if self.board else '',
                               motion_epoch=self.board.motion_epoch if self.board else 0)

    def _accept(self, request):
        return self._accept_area(request, 'coarse')

    def _accept_raw(self, request):
        return self._accept_area(request, 'raw')

    def _accept_storage(self, request):
        return self._accept_area(request, 'storage')

    def _accept_area(self, request, area):
        raw = area == 'raw'
        stage = (4 if request.batch == 1 else 10) if raw else (6 if request.batch == 1 else 12)
        expected_operation = 'RAW_PICK' if raw else 'COARSE_PROCESS'
        if area == 'storage':
            stage = 8 if request.batch == 1 else 14
            expected_operation = 'STORAGE_PLACE' if request.batch == 1 else 'STORAGE_STACK'
        first_layer = {row.color: row.position for row in request.first_layer}
        mapping_ok = (request.operation != 'STORAGE_STACK' or
                      (len(request.first_layer) == 3 and len(first_layer) == 3 and
                       sorted(first_layer.values()) == [1, 2, 3] and
                       [first_layer.get(color) for color in request.colors] == list(request.positions)))
        valid = (request.operation == expected_operation and request.batch in (1, 2) and mapping_ok and
                 (not request.defer_reference_return or expected_operation in ('RAW_PICK', 'COARSE_PROCESS', 'STORAGE_PLACE')) and
                 request.stage == stage and request.task_revision == 1 and
                 request.task_id and request.enable_motion and len(request.colors) == 3 and
                 len(set(request.colors)) == 3 and all(1 <= c <= 6 for c in request.colors) and
                 (not request.positions if raw else sorted(request.positions) == [1, 2, 3]) and
                 self.contract.authorized(request.run_id))
        if not valid or self._reserved or self.goal or not self._config_ready(request.operation) or not self._healthy() or not self._idle():
            self.get_logger().warning(f'拒绝 {area} 任务：检查任务、配置、视觉、控制权及机构状态')
            return GoalResponse.REJECT
        self._reserved = True
        return GoalResponse.ACCEPT

    async def _execute(self, goal):
        self.goal = goal
        self._done = Future(executor=self.executor)
        self._reserved = False
        req = goal.request
        self.p = self.configs[req.operation]
        self.area = {'RAW_PICK': 'raw', 'COARSE_PROCESS': 'coarse',
                     'STORAGE_PLACE': 'storage', 'STORAGE_STACK': 'storage'}[req.operation]
        self._link_paused_at = None
        self._reported_state = ''
        self._binding = (self.board.odom_session_id, self.board.motion_epoch)
        args = (self.p, req.task_id, req.colors, req.positions, time.monotonic())
        self.task = (CoarseTask(*args, defer_reference_return=req.defer_reference_return) if req.operation == 'COARSE_PROCESS' else
                     RawTask(self.p, req.task_id, req.colors, args[-1],
                             defer_reference_return=req.defer_reference_return) if req.operation == 'RAW_PICK' else
                     StorageTask(*args, req.operation, {row.color: row.position for row in req.first_layer},
                                 defer_reference_return=req.defer_reference_return))
        self._submitted.clear()
        self._arm_future = None
        self._abort_id = ''
        self._sequence = 0
        self._stowed = False
        self.vision.request(True, area='raw' if self.area == 'raw' else 'coarse')
        result = await self._done
        if goal.is_cancel_requested:
            goal.canceled()
        elif result.outcome == 'COMPLETED':
            goal.succeed()
        else:
            goal.abort()
        self.last_task_id = req.task_id
        self.goal = self._done = None
        return result

    def _cancel(self, goal):
        if goal is self.goal:
            self._stop('Action取消', failed=False)
        return CancelResponse.ACCEPT

    def _reset(self, request):
        del request
        self.task = None
        self.last_task_id = ''
        self._abort_id = ''
        self._expected_arm_id = ''
        self._stowed = True

    def _stop(self, reason, failed=True):
        if self.task and not self.task.finished:
            self.task.stop(time.monotonic(), reason, failed)
        self.vision.request(False)
        if self._binding:
            self._servo.publish(None, self._binding)
        if self.goal:
            self._publish_control()
        # The last published idle may precede a just-transmitted arm request.
        if not self._abort_id and (self.goal is not None or not self._idle() or self._arm_future is not None):
            self._abort_id = uuid.uuid4().hex
            if self._arm_client.service_is_ready():
                self._expected_arm_id = self._abort_id
                self._arm_future = (self._arm_client.call_async(ArmCommand.Request(
                    command_id=self._abort_id, cmd=0)), time.monotonic(), self._abort_id)

    def _tick(self):
        self._check_arm_future()
        if not self.goal or self._done.done():
            return
        task, now = self.task, time.monotonic()
        try:
            if not task.finished:
                paused = getattr(self, '_link_paused_at', None)
                if (self._binding == (self.board.odom_session_id, self.board.motion_epoch) and
                        (communication_lost(self.board) or
                         (paused is not None and not self.board.fault_reason and not self._healthy()))):
                    if paused is None:
                        self._link_paused_at = now
                    task.velocity, task.arm_velocity = (0., 0., 0.), (0., 0.)
                    self._servo.publish(None, self._binding)
                    self._publish_control()
                    self._feedback()
                    return
                if paused is not None:
                    resume_task(task, now-paused)
                    self._link_paused_at = None
                if not self._healthy() or self._binding != (self.board.odom_session_id, self.board.motion_epoch):
                    ages = tuple(round(now-at, 3) for at in (self._board_at, self._odom_at, self._arm_at))
                    raise RuntimeError('C板/里程/机构反馈异常或坐标控制权改变：'
                                       f'C板={self.board.fault_reason or "无故障码"}，'
                                       f'机构={self.arm.state} {self.arm.reason}，'
                                       f'反馈年龄(C板/里程/机构)={ages}s，'
                                       f'坐标绑定={self._binding}/{(self.board.odom_session_id, self.board.motion_epoch)}')
                if self.vision.error:
                    raise RuntimeError(self.vision.error)
                pose = Pose(self.odom.x_m, self.odom.y_m, self.odom.yaw_rad, self._odom_at,
                            self.odom.vx_mps, self.odom.vy_mps)
                self.vision.request(True, task.vision_kind, task.vision_color,
                                    area='raw' if self.area == 'raw' else 'coarse',
                                    geometry_target=task.vision_target)
                observation = self.vision.latest()
                now = time.monotonic()
                observation = self._vision_gate.filter(task, observation, now, self.area)
                if not task.vision_kind or observation is not None:
                    task.tick(now, pose, observation, self.arm,
                              GantryPose.from_status(self.board, self._board_at))
                if now-self._last_diagnostic >= 1.:
                    self._last_diagnostic = now
                    frame = ((observation.sequence, now-observation.captured_at, observation.center_sources)
                             if observation else None)
                    self.get_logger().info(f'CONTROL {task.state} frame(seq,age,source)={frame} '
                                           f'data={task.diagnostic} velocity={task.velocity}')
                # Publish zero before submitting an arm request, including while ACK is pending.
                if task.arm_request is not None:
                    self._publish_control()
                # Zero-speed visual waits retain the stream; fixed actions close it.
                self._servo.publish(task, self._binding, hold_zero=self.area == 'storage' and
                                    task.state in ('SCAN_FIRST_LAYER','ALIGN_RING','ALIGN_MATERIAL'))
                self._submit_task_arm()
                self.vision.request(not task.finished, task.vision_kind, task.vision_color,
                                    area='raw' if self.area == 'raw' else 'coarse',
                                    geometry_target=task.vision_target)
            self._publish_control()
            self._feedback()
            if task.finished:
                self._finish()
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(str(error))
            self._stop(str(error))

    def _submit_task_arm(self):
        request = self.task.arm_request
        if request is None or request['command_id'] in self._submitted:
            return
        if not self._idle():
            if time.monotonic()-self.task.entered_at > self.p['execution']['arm_request_timeout_s']:
                raise RuntimeError('等待机构空闲/停轴释放超时')
            return
        binding = dict(odom_session_id=self._binding[0], motion_epoch=self._binding[1])
        self._expected_arm_id = request['command_id']
        if request['cmd'] is None:
            message = JogArm.Request(command_id=request['command_id'], **binding,
                                     ud_rpm=-self.p['arm']['lower_speed_rpm'],
                                     duration_s=self.p['arm']['lower_duration_s'])
            future = self._jog_client.call_async(message)
        else:
            future = self._arm_client.call_async(ArmCommand.Request(**request, **binding))
        self._submitted.add(request['command_id'])
        self._arm_future = (future, time.monotonic(), request['command_id'])

    def _check_arm_future(self):
        if self._arm_future is None:
            return
        future, at, command_id = self._arm_future
        if not future.done() and time.monotonic()-at <= self.p['execution']['arm_request_timeout_s']:
            return
        self._arm_future = None
        try:
            if not future.done():
                raise RuntimeError('机械臂服务结果未知，禁止重发')
            response = future.result()
            if not response.accepted:
                raise RuntimeError('机械臂请求被拒绝：'+response.reason)
        except Exception as error:
            self.get_logger().error(str(error))
            self._stop(str(error))

    def _publish_control(self):
        task = self.task
        vx, vy, wz = task.velocity
        command = ControlCommand(vx_mps=vx, vy_mps=vy, omega_deg_s=math.degrees(wz),
                                 odom_session_id=self._binding[0], motion_epoch=self._binding[1])
        self._sequence += 1
        message = RouteControl(run_id=self.goal.request.run_id, request_id=task.task_id,
                               executor_session_id=self.contract.session_id, source='REAL',
                               sequence=self._sequence, command=command,
                               state='EXECUTING')
        message.header.stamp = self.get_clock().now().to_msg()
        self._control_pubs[self.area].publish(message)

    def _records(self):
        req = self.goal.request
        return [WorkStepRecord(run_id=req.run_id, task_id=req.task_id, step_id=f'{op}:{i}',
                               batch=req.batch, item_index=i, operation=op, color=color,
                               position=position, controller_completed=True, visually_verified=False)
                for op, i, color, position in self.task.confirmed]

    def _feedback(self):
        task = self.task
        item = min(task.item, 2)
        if task.state != self._reported_state:
            self.get_logger().info(f'{task.task_id} {task.state} item={item+1} '
                                   f'reference={task.reference} diagnostic={task.diagnostic} '
                                   f'angles={getattr(task, "entry_alignment", "NOT_RUN")}/{task.final_alignment}')
            self._reported_state = task.state
        self.goal.publish_feedback(RunWorkBatch.Feedback(
            run_id=self.goal.request.run_id, task_id=task.task_id, phase=task.state,
            item_index=item+1, color=task.colors[item], position=task.positions[item],
            arm_pending=bool(self.arm.pending or self._arm_future or
                             (task.arm_request and not self._idle())), confirmed_steps=self._records(),
            reason=task.reason))

    def _finish(self):
        task, req = self.task, self.goal.request
        completed = task.state == 'DONE'
        coarse = req.operation == 'COARSE_PROCESS'
        ready = completed and self._idle()
        if self.area in ('raw', 'coarse', 'storage'):
            motion = self.p['base_motion' if self.area == 'raw' else 'base_servo']
            ready = (ready and not self._servo.stream_id and task.velocity == (0.,0.,0.) and
                     math.hypot(self.odom.vx_mps,self.odom.vy_mps) <= motion['stop_speed_mps'])
        self._stowed = ready
        reason = task.reason
        if completed and task.final_alignment == 'TIMEOUT_SKIPPED':
            reason = f'最终视觉角度校准达 {self.p["angle_alignment"]["timeout_s"]:g} 秒上限，按约定跳过；角度未确认达标'
        result = RunWorkBatch.Result(
            run_id=req.run_id, task_id=req.task_id, task_revision=req.task_revision,
            executor_session_id=self.contract.session_id, source='REAL',
            outcome='COMPLETED' if completed else ('STOPPED' if task.state == 'STOPPED' else 'FAILED'),
            confirmed_steps=self._records(), all_items_completed=len(task.confirmed) == (6 if coarse else 3),
            reference_restored=task.returned,
            reference_return_deferred=task.reference_return_deferred,
            reference_return_timed_out=task.reference_return_timed_out,
            motion_released=True, arm_idle=self._idle(), travel_ready=ready,
            first_layer_records=([FirstLayerRecord(color=color, position=position)
                                  for op, i, color, position in task.confirmed]
                                 if req.operation == 'STORAGE_PLACE' else []),
            final_alignment_outcome=task.final_alignment, reason=reason)
        self.vision.request(False)
        self._done.set_result(result)

    def destroy_node(self):
        self._stop('节点退出')
        self.vision.close()
        self._action.destroy()
        self._raw_action.destroy()
        self._storage_action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CoarseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
