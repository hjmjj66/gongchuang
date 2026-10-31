"""Plan routes and execute them from fixed-frame XY and continuous yaw feedback."""

import math
import time
import uuid

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agv_interfaces.msg import (
    BlockedEdges, CBoardStatus, ControlCommand, OdometryFeedback, RouteControl, RouteExecutionStatus,
    RoutePlan, RouteStep, ExecutorRuntime, Edge, RouteExecutionProgress,
)
from agv_interfaces.srv import PlanRoute

from .motion import AnglePID, PositionPID, TRANSLATIONS, merge_steps
from .odometry import body_to_fixed, projection
from .planner import EDGES, HEADINGS, Opcode, Topology, check_node, normalize_heading, parking_step
from .executor_contract import ExecutorContract


class _DuplicateRequestId(Exception):
    pass


class RoutePlannerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('route_planner_node', **kwargs)
        self._blocked = frozenset()
        self._map_revision = 0
        self._map_ready = False
        self._active = False
        self._request_id = ''
        self._control_sequence = 0
        self._accepted_request_ids = set()
        self._segments = ()
        self._segment_index = 0
        self._segment_baseline = None
        self._last_odom_received = None
        self._odom_fault = None
        self._position = None
        self._target_position = None
        self._body_velocity = (0.0, 0.0)
        self._direction = None
        self._binding = None
        self._task_binding = None
        self._board_binding = None
        self._board_ready = False
        self._board_received = None
        self._board_timeout = float(self._param('board_status_timeout_s', 0.2))
        self._context_invalid = False
        self._yaw_deg = None
        self._initial_yaw_deg = None
        self._target_yaw_deg = None
        self._wz_deg_s = 0.0
        self._last_task_id = ''
        self._last_state = ''
        self._last_reason = ''
        self._segment_from_node = 0
        self._segment_start_heading = ''
        self._progress_state = 'IDLE'

        area = self._param('start_area', 'left')
        if area not in ('left', 'right'):
            raise ValueError("start_area must be 'left' or 'right'")
        self._start_area = area
        self._home_node = 1 if area == 'left' else 3
        self._initial_heading = 'DOWN' if area == 'left' else 'UP'
        self._current_node = self._home_node
        self._current_heading = self._initial_heading
        self._heading_reference = self._initial_heading
        self._position_known = True
        self._work_headings = {
            node: self._configured_heading(f'work_heading.node_{node}')
            for node in (2, 4, 6, 8)
        }

        default_length = self._param('default_edge_length_mm', 1200)
        lengths = {}
        for a, b in EDGES:
            key = (min(a, b), max(a, b))
            lengths[key] = self._param(f'edge_length_mm.{key[0]}_{key[1]}', default_length)
        self._topology = Topology(default_length, lengths)
        self._require_snapshot = self._param('require_map_snapshot', True)

        self._translation_kp = float(self._param('translation_pid_kp', 2.0))
        self._translation_ki = float(self._param('translation_pid_ki', 0.0))
        self._translation_kd = float(self._param('translation_pid_kd', 0.1))
        self._vx_max = float(self._param('translation_max_vx_mps', 0.7))
        self._vy_max = float(self._param('translation_max_vy_mps', 0.4))
        self._translation_tolerance_mm = float(self._param('translation_arrival_tolerance_mm', 3.0))
        self._angle_kp = float(self._param('angle_pid_kp', 2.0))
        self._angle_ki = float(self._param('angle_pid_ki', 0.0))
        self._angle_kd = float(self._param('angle_pid_kd', 0.1))
        self._w_max = float(self._param('angle_max_speed_deg_s', 120.0))
        self._turn_tolerance_deg = float(self._param('turn_arrival_tolerance_deg', 0.2))
        self._odom_timeout = float(self._param('odometry_timeout_s', 0.5))
        self._motion_progress_timeout = float(self._param('motion_progress_timeout_s', 2.0))
        self._control_rate = float(self._param('control_rate_hz', 50.0))
        self._validate_motion_config()
        self._angle_pid = AnglePID(self._angle_kp, self._angle_ki, self._angle_kd,
                                   self._w_max, self._turn_tolerance_deg)
        self._position_pid = PositionPID(self._translation_kp, self._translation_ki,
                                         self._translation_kd, (self._vx_max, self._vy_max),
                                         self._translation_tolerance_mm / 1000.0)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(
            RoutePlan, self._param('route_topic', '/route/path'), reliable
        )
        self._control_publisher = self.create_publisher(
            RouteControl, self._param('control_topic', '/route/control'), reliable
        )
        if self._control_publisher.topic_name == '/C/controll':
            raise ValueError('Route feedback must use an internal topic; /C/controll belongs to mission')
        self._status_publisher = self.create_publisher(
            RouteExecutionStatus,
            self._param('execution_status_topic', '/route/execution_status'), reliable,
        )
        self._map_sub = self.create_subscription(
            BlockedEdges, self._param('blocked_edges_topic', '/map/blocked_edges'),
            self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self._odom_sub = self.create_subscription(
            OdometryFeedback, self._param('odometry_topic', '/C/odometry'),
            self._on_odometry, reliable,
        )
        self._board_sub = self.create_subscription(
            CBoardStatus, self._param('board_status_topic', '/C/status'),
            self._on_board, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._service = self.create_service(
            PlanRoute, self._param('plan_service', '/route/plan'), self._on_plan
        )
        self._contract = ExecutorContract(
            self, '/route', self._param('result_source', 'REAL'),
            float(self._param('mission_lease_timeout_s', 2.0)),
            self._runtime, self._stop_execution, self._reset_context,
            debug_allow_any_pose=self._param('debug_allow_any_pose', False),
        )
        self._progress_pub = None
        if self._param('debug_publish_progress', False):
            self._progress_pub = self.create_publisher(
                RouteExecutionProgress,
                self._param('debug_progress_topic', '/route/progress'), reliable)
            self._progress_timer = self.create_timer(0.1, self._publish_progress)
        self._timer = self.create_timer(1.0 / self._control_rate, self._tick)
        self.get_logger().info(
            f'Ready: start_area={area} (node {self._home_node}, facing {self._initial_heading}); '
            f'control feedback={self._control_publisher.topic_name}; mission forwards base velocities.'
        )

    def _param(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def _configured_heading(self, name):
        value = self._param(name, 'UNSET')
        return None if value == 'UNSET' else normalize_heading(value)

    def _validate_motion_config(self):
        positive = (self._vx_max, self._vy_max, self._w_max,
                    self._odom_timeout, self._motion_progress_timeout, self._control_rate, self._board_timeout)
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError('motion speed limits, timeouts and rates must be positive')
        timer_period_ns = 1e9 / self._control_rate
        if not math.isfinite(timer_period_ns) or not 1 <= timer_period_ns < 2**63:
            raise ValueError('control rate must produce a representable timer period of at least 1 ns')
        for name, gains in (('angle', (self._angle_kp, self._angle_ki, self._angle_kd)),
                            ('translation', (self._translation_kp, self._translation_ki,
                                             self._translation_kd))):
            if not all(math.isfinite(value) and value >= 0 for value in gains) or gains[0] <= 0:
                raise ValueError(f'{name} PID gains must be finite and nonnegative; Kp must be positive')
        if (not math.isfinite(self._translation_tolerance_mm)
                or not math.isfinite(self._turn_tolerance_deg)
                or self._translation_tolerance_mm < 0 or self._turn_tolerance_deg < 0):
            raise ValueError('arrival tolerances must be finite and nonnegative')
        if (self._translation_tolerance_mm >= min(self._topology.lengths.values())
                or self._turn_tolerance_deg >= 90.0):
            raise ValueError('arrival tolerances must be smaller than one edge or one quarter turn')

    def _on_map(self, message):
        if message.run_id != self._contract.run_id:
            self.get_logger().warning('Map rejected: run_id mismatch', throttle_duration_sec=5.0)
            return
        if self._active:
            if message.map_revision != self._map_revision:
                self.get_logger().warning('Map changes rejected during route execution')
            return
        try:
            if message.map_revision == 0:
                raise ValueError('map_revision must be positive')
            blocked = self._topology.normalize_blocked((edge.a, edge.b) for edge in message.edges)
            if self._map_ready and message.map_revision < self._map_revision:
                raise ValueError('stale map_revision')
            if self._map_ready and message.map_revision == self._map_revision:
                if blocked != self._blocked:
                    raise ValueError('same map_revision has different blocked edges')
                return
        except ValueError as exc:
            self.get_logger().warning(f'Map snapshot rejected; keeping previous map: {exc}')
            return
        self._blocked = blocked
        self._map_revision = message.map_revision
        self._map_ready = True

    def _invalidate_context(self, reason):
        self._abort(reason)
        self._position_known = False
        self._context_invalid = True
        self._initial_yaw_deg = None
        self._target_yaw_deg = None
        self._target_position = None
        self._segment_baseline = None

    def _on_board(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= self._board_timeout:
            return
        binding = (message.odom_session_id, message.motion_epoch)
        changed = self._board_binding is not None and binding != self._board_binding
        self._board_binding = binding
        self._board_received = time.monotonic()-age
        self._board_ready = bool(message.motion_allowed and message.origin_initialized)
        if changed or (not self._board_ready and (self._active or self._binding is not None)):
            self._invalidate_context('C-board permission/origin changed; reinitialize map pose')

    def _on_odometry(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= self._odom_timeout:
            return
        binding = (message.odom_session_id, message.motion_epoch)
        values = (message.x_m, message.y_m, message.yaw_rad, message.yaw_deg,
                  message.vx_mps, message.vy_mps, message.wz_rad_s)
        if self._binding is not None and binding != self._binding:
            self._invalidate_context('odometry coordinate/permission changed; reinitialize map pose')
        self._binding = binding
        if (not message.valid or not message.origin_initialized or not binding[0]
                or not all(math.isfinite(v) for v in values)):
            self._last_odom_received = None
            self._odom_fault = 'invalid XY odometry'
            self._invalidate_context(self._odom_fault)
            return
        self._odom_fault = None
        self._position = (float(message.x_m), float(message.y_m))
        self._body_velocity = (float(message.vx_mps), float(message.vy_mps))
        self._yaw_deg = float(message.yaw_deg)
        self._wz_deg_s = math.degrees(message.wz_rad_s)
        self._last_odom_received = time.monotonic()-age
        if not self._context_invalid:
            if self._initial_yaw_deg is None:
                self._initial_yaw_deg = self._yaw_deg
                self._target_yaw_deg = self._yaw_deg
                self._target_position = self._position
            delta = self._yaw_deg-self._initial_yaw_deg
            quarter_turns = int(round(delta/90.0))
            base = HEADINGS.index(self._heading_reference)
            self._current_heading = HEADINGS[(base-quarter_turns) % 4]

    def _feedback_ready(self):
        now = time.monotonic()
        return bool(self._last_odom_received is not None
                    and now-self._last_odom_received <= self._odom_timeout
                    and not self._odom_fault and self._board_ready
                    and self._board_received is not None
                    and now-self._board_received <= self._board_timeout
                    and self._binding == self._board_binding)

    def _resolve_request(self, request):
        if not self._contract.authorized(request.run_id):
            raise ValueError('run_id mismatch or mission lease expired')
        if request.run_id and (request.start != 0 or request.current_heading):
            raise ValueError('Competition requests must use confirmed start=0 and feedback heading')
        if not 0 <= request.stage <= 17:
            raise ValueError('stage must be in 0..17')
        if self._context_invalid:
            raise ValueError('Map pose invalidated; reset executor context with confirmed pose')
        if request.start == 0 and not self._position_known:
            raise ValueError('Position uncertain after abort; resynchronize and supply explicit start/heading')
        start = self._current_node if request.start == 0 else check_node(request.start)
        goal = self._home_node if request.goal == 0 else check_node(request.goal)
        if request.current_heading:
            heading = normalize_heading(request.current_heading)
        elif request.start == 0:
            heading = self._current_heading
        else:
            raise ValueError('current_heading required when start is specified explicitly')
        goal_heading = None
        if request.align_at_goal:
            if goal not in self._work_headings:
                raise ValueError('align_at_goal requires a work node: 2, 4, 6 or 8')
            goal_heading = (normalize_heading(request.goal_heading) if request.goal_heading
                            else self._work_headings[goal])
            if goal_heading is None:
                raise ValueError(f'work_heading.node_{goal} is UNSET; configure or supply goal_heading')
        elif request.goal_heading:
            raise ValueError('goal_heading must be empty when align_at_goal=false')
        if request.parking_maneuver:
            if (request.parking_maneuver not in ('EXIT', 'ENTER') or start != goal
                    or goal not in (1, 3) or request.align_at_goal):
                raise ValueError('parking maneuver requires EXIT/ENTER, start=goal=1/3 and no alignment')
            if request.parking_offset_mm <= self._translation_tolerance_mm:
                raise ValueError('parking offset must exceed translation arrival tolerance')
        elif request.parking_offset_mm:
            raise ValueError('parking_offset_mm requires a parking maneuver')
        return start, goal, heading, goal_heading

    def _on_plan(self, request, response):
        plan = RoutePlan()
        plan.run_id = request.run_id
        plan.request_id = request.request_id or str(uuid.uuid4())
        plan.stage = request.stage
        plan.start = self._current_node if request.start == 0 else request.start
        plan.goal = self._home_node if request.goal == 0 else request.goal
        plan.map_revision = self._map_revision
        try:
            if plan.request_id in self._accepted_request_ids:
                plan.status = 'duplicate_request_id'
                plan.reason = 'This request_id was already accepted; use a new ID for a new route'
                raise _DuplicateRequestId
            start, goal, heading, goal_heading = self._resolve_request(request)
            plan.start, plan.goal, plan.start_heading = start, goal, heading
            if self._active:
                plan.status = 'execution_in_progress'
                plan.reason = 'A previous route is still executing'
            elif (self._require_snapshot and not self._map_ready
                  and request.parking_maneuver != 'EXIT'):
                plan.status = 'map_not_ready'
                plan.reason = 'No valid map snapshot received; publish an explicit snapshot first'
            elif self._odom_fault:
                plan.status = 'odometry_invalid'
                plan.reason = self._odom_fault
            elif not self._feedback_ready():
                plan.status = 'odometry_not_ready'
                plan.reason = 'No fresh odometry feedback; route execution was not started'
            else:
                result = self._topology.shortest_path(start, goal, self._blocked)
                if result is None:
                    plan.status = 'no_path'
                    plan.reason = 'No route between endpoints with the current blocked edges'
                else:
                    steps = ((parking_step(goal, heading, request.parking_maneuver,
                                           request.parking_offset_mm),)
                             if request.parking_maneuver else
                             self._topology.build_steps(result.nodes, heading, goal_heading))
                    plan.actions = [self._to_message_step(index, step)
                                    for index, step in enumerate(steps)]
                    plan.nodes = list(result.nodes)
                    plan.total_distance_mm = (round(request.parking_offset_mm * math.sqrt(2.0))
                                              if request.parking_maneuver else result.total_distance_mm)
                    plan.direction_changes = result.direction_changes
                    plan.end_heading = goal_heading or heading
                    plan.status = 'ok'
                    self._current_node = start
                    self._current_heading = heading
                    self._position_known = True
                    if request.start != 0 or request.current_heading:
                        self._heading_reference = heading
                        self._initial_yaw_deg = self._yaw_deg
                        self._target_yaw_deg = self._yaw_deg
                        self._target_position = self._position
                    self._remember_request_id(plan.request_id)
                    self._start_execution(plan.request_id, steps, goal)
        except _DuplicateRequestId:
            pass
        except ValueError as exc:
            plan.status = 'invalid_request'
            plan.reason = str(exc)
        response.plan = plan
        self._publisher.publish(plan)
        state = 'EXECUTING' if plan.status == 'ok' and self._active else 'COMPLETED'
        if plan.status != 'ok':
            state = 'REJECTED'
        self._publish_status(plan.request_id, state, 0, plan.reason)
        self.get_logger().info(
            f'route request_id={plan.request_id} {plan.start}->{plan.goal} '
            f'status={plan.status} nodes={list(plan.nodes)} '
            f'actions={[step.opcode for step in plan.actions]} reason={plan.reason}'
        )
        return response

    def _remember_request_id(self, request_id):
        if not request_id:
            return
        self._accepted_request_ids.add(request_id)

    @staticmethod
    def _to_message_step(index, step):
        action = RouteStep()
        action.step_index = index
        action.opcode = int(step.opcode)
        action.from_node = step.from_node
        action.to_node = step.to_node
        action.distance_mm = step.distance_mm
        action.target_heading = step.target_heading
        return action

    def _start_execution(self, request_id, steps, goal):
        self._request_id = request_id
        self._task_binding = self._binding
        self._control_sequence = 0
        self._segments = merge_steps(steps)
        self._segment_index = 0
        self._segment_baseline = None
        self._segment_from_node = self._current_node
        self._segment_start_heading = self._current_heading
        self._progress_state = 'EXECUTING' if self._segments else 'COMPLETED'
        self._active = bool(self._segments)
        self._execution_goal = goal
        if not self._active:
            self._current_node = goal
            self._publish_zero()

    def _tick(self):
        if not self._active:
            return
        if not self._feedback_ready():
            self._invalidate_context('C-board feedback/permission timed out')
            return
        segment = self._segments[self._segment_index]
        if self._segment_baseline is None:
            self._segment_from_node = self._current_node
            self._segment_start_heading = self._current_heading
            self._segment_baseline = (*self._position, self._target_yaw_deg)
            self._direction = (body_to_fixed(*TRANSLATIONS[segment.opcode], math.radians(self._target_yaw_deg))
                               if segment.opcode in TRANSLATIONS else None)
            if segment.opcode in TRANSLATIONS:
                self._target_position = tuple(position + direction * segment.amount / 1000.0
                                              for position, direction in
                                              zip(self._target_position, self._direction))
            else:
                sign = 1.0 if segment.opcode == Opcode.TURN_LEFT_90 else -1.0
                self._target_yaw_deg += sign * segment.amount
            self._angle_pid.reset()
            self._position_pid.reset()
            self._angle_control_time = None
            self._angle_progress_error = None
            self._position_progress_error = [None, None]
            self._position_progress_time = [time.monotonic(), time.monotonic()]
        _, _, yaw_base = self._segment_baseline
        error = self._target_yaw_deg - self._yaw_deg
        if not math.isfinite(error):
            self._abort('invalid yaw difference during segment')
            return
        angle_done = abs(error) <= self._turn_tolerance_deg
        if self._check_angle_progress(abs(error)):
            self._abort('yaw error made no progress during segment')
            return
        now = time.monotonic()
        dt = (1.0 / self._control_rate if self._angle_control_time is None
              else now - self._angle_control_time)
        self._angle_control_time = now
        omega = self._angle_pid.update(error, self._wz_deg_s, dt)
        if segment.opcode in TRANSLATIONS:
            velocities, errors, done = self._translation_motion(dt)
            if self._check_position_progress(errors):
                self._abort('XY position error made no progress during segment')
                return
            velocities = body_to_fixed(*velocities, math.radians(yaw_base - self._yaw_deg))
            # Heading correction rotates XY, so enforce the limits again in the current body frame.
            velocities = tuple(max(-limit, min(limit, value)) for value, limit in
                               zip(velocities, (self._vx_max, self._vy_max)))
            self._publish_control(*velocities, omega)
            done = done and angle_done
        else:
            self._publish_control(0.0, 0.0, omega)
            done = angle_done
        if done:
            self._publish_zero()
            self._current_node = segment.end_node
            self._current_heading = segment.target_heading
            self._segment_index += 1
            if self._segment_index >= len(self._segments):
                self._active = False
                self._current_node = self._execution_goal
                self._publish_status(self._request_id, 'COMPLETED',
                                     self._segment_index - 1, '')
            else:
                self._segment_baseline = None
                self._publish_status(self._request_id, 'EXECUTING', self._segment_index, '')

    def _translation_motion(self, dt):
        yaw_base = self._segment_baseline[2]
        errors = body_to_fixed(self._target_position[0] - self._position[0],
                               self._target_position[1] - self._position[1],
                               -math.radians(yaw_base))
        velocities = body_to_fixed(*self._body_velocity, math.radians(self._yaw_deg - yaw_base))
        command = self._position_pid.update(errors, velocities, dt)
        done = all(abs(error) <= self._position_pid.tolerance for error in errors)
        return command, errors, done

    def _check_position_progress(self, errors):
        now = time.monotonic()
        threshold = min(0.1, self._translation_tolerance_mm)
        for axis, error in enumerate(errors):
            error_mm = abs(error) * 1000.0
            if error_mm <= self._translation_tolerance_mm:
                self._position_progress_error[axis] = None
            elif (self._position_progress_error[axis] is None
                  or error_mm < self._position_progress_error[axis] - threshold):
                self._position_progress_error[axis] = error_mm
                self._position_progress_time[axis] = now
            elif now - self._position_progress_time[axis] > self._motion_progress_timeout:
                return True
        return False

    def _check_angle_progress(self, error):
        now = time.monotonic()
        if error <= self._turn_tolerance_deg:
            self._angle_progress_error = None
        elif (self._angle_progress_error is None
              or error < self._angle_progress_error - min(0.1, self._turn_tolerance_deg)):
            self._angle_progress_error = error
            self._angle_progress_time = now
        else:
            return now - self._angle_progress_time > self._motion_progress_timeout
        return False

    def _publish_control(self, vx, vy, omega, state='EXECUTING', reason=''):
        self._control_sequence += 1
        feedback = RouteControl(
            run_id=self._contract.run_id, executor_session_id=self._contract.session_id,
            source=self._contract.source, request_id=self._request_id,
            sequence=self._control_sequence, state=state, reason=reason,
            command=ControlCommand(vx_mps=float(vx), vy_mps=float(vy), omega_deg_s=float(omega),
                                   odom_session_id=(self._task_binding or self._binding or ('', 0))[0],
                                   motion_epoch=(self._task_binding or self._binding or ('', 0))[1]),
        )
        feedback.header.stamp = self.get_clock().now().to_msg()
        self._control_publisher.publish(feedback)

    def _publish_zero(self):
        self._publish_control(0.0, 0.0, 0.0)

    def _publish_status(self, request_id, state, index, reason):
        status = RouteExecutionStatus()
        status.run_id = self._contract.run_id
        status.executor_session_id = self._contract.session_id
        status.source = self._contract.source
        status.request_id = request_id
        status.state = state
        status.segment_index = index
        status.reason = reason
        self._status_publisher.publish(status)
        # Carry terminal state and zero speed together, independent of status-topic ordering.
        # Rejections of unrelated requests must never stop the accepted task.
        if request_id == self._request_id and state in ('COMPLETED', 'ABORTED'):
            self._publish_control(0.0, 0.0, 0.0, state, reason)
        # Rejected unrelated requests must not overwrite the accepted task's terminal state.
        if state != 'REJECTED':
            self._last_task_id, self._last_state, self._last_reason = request_id, state, reason
            self._progress_state = state
            self._publish_progress()

    def _publish_progress(self):
        if self._progress_pub is None or not self._request_id:
            return
        # Snapshot on the same single-threaded executor as segment advancement.
        msg = RouteExecutionProgress()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self._contract.run_id
        msg.request_id = self._request_id
        msg.executor_session_id = self._contract.session_id
        msg.state = self._progress_state
        msg.feedback_valid = bool(self._last_odom_received is not None and
                                  time.monotonic() - self._last_odom_received <= self._odom_timeout
                                  and not self._odom_fault)
        msg.x_m, msg.y_m = self._position or (0., 0.)
        msg.odom_session_id, msg.motion_epoch = self._binding or ('', 0)
        msg.baseline_x_m, msg.baseline_y_m = msg.x_m, msg.y_m
        msg.yaw_deg = self._yaw_deg or 0.0
        msg.baseline_yaw_deg = msg.yaw_deg
        msg.from_node = self._segment_from_node
        msg.start_heading = self._segment_start_heading
        msg.to_node = self._current_node
        if self._segments:
            index = min(self._segment_index, len(self._segments) - 1)
            msg.segment_index = index
            segment = self._segments[index]
            msg.opcode, msg.amount, msg.to_node = int(segment.opcode), segment.amount, segment.end_node
            baseline = self._segment_baseline
            if baseline is not None:
                msg.baseline_x_m, msg.baseline_y_m, msg.baseline_yaw_deg = baseline
                if segment.opcode in TRANSLATIONS:
                    msg.direction_x, msg.direction_y = self._direction
                    msg.signed_progress_mm, msg.cross_track_mm = projection(
                        baseline[:2], self._position, self._direction)
                    msg.progress = max(0.0, msg.signed_progress_mm)
                else:
                    sign = 1.0 if segment.opcode == Opcode.TURN_LEFT_90 else -1.0
                    msg.progress = max(0.0, (msg.yaw_deg - baseline[2]) * sign)
            elif not self._active and self._progress_state == 'COMPLETED':
                msg.from_node = segment.end_node
                msg.progress = segment.amount
            elif self._active:
                # A new segment has not taken its first odometry baseline yet.
                msg.from_node = self._current_node
                msg.start_heading = self._current_heading
        self._progress_pub.publish(msg)

    def _runtime(self):
        state = ExecutorRuntime()
        state.busy = self._active
        state.quiescent = not self._active
        state.motion_released = not self._active
        state.travel_ready = not self._active
        state.config_ready = all(self._work_headings.values())
        state.active_task_id = self._request_id if self._active else ''
        state.last_task_id, state.last_state = self._last_task_id, self._last_state
        state.reason = self._last_reason
        state.home_node = self._home_node
        state.current_node = self._current_node
        state.position_known = self._position_known and not self._active
        state.current_heading = self._current_heading
        state.yaw_deg = self._yaw_deg or 0.0
        state.odometry_ready = self._feedback_ready()
        state.odom_session_id, state.motion_epoch = self._binding or ('', 0)
        state.heading_known = state.odometry_ready and not self._context_invalid
        state.map_ready, state.map_revision = self._map_ready, self._map_revision
        state.blocked_edges = [Edge(a=a, b=b) for a, b in sorted(self._blocked)]
        return state

    def _stop_execution(self, reason):
        self._abort(reason)

    def _reset_context(self, request):
        self._blocked = frozenset()
        self._map_revision, self._map_ready = 0, False
        self._home_node = request.home_node
        self._initial_heading = request.initial_heading
        self._current_node, self._current_heading = request.home_node, request.initial_heading
        self._heading_reference = request.initial_heading
        self._initial_yaw_deg = None
        self._target_yaw_deg = None
        self._target_position = None
        self._angle_pid.reset()
        self._position_pid.reset()
        self._position_known = True
        self._context_invalid = False
        self._task_binding = None
        self._odom_fault = None
        # Await a fresh sample after the confirmed map-pose reset.
        self._last_odom_received = None
        self._request_id = ''
        self._control_sequence = 0
        self._segments = ()
        self._progress_state = 'IDLE'
        self._last_task_id = self._last_state = self._last_reason = ''

    def _abort(self, reason):
        if self._active:
            self._publish_zero()
            self._active = False
            self._position_known = False
            self._publish_status(self._request_id, 'ABORTED', self._segment_index, reason)
            self.get_logger().error(f'Route {self._request_id} aborted: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoutePlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node._active and rclpy.ok():
                node._publish_zero()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
