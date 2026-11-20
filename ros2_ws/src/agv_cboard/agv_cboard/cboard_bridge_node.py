"""ROS2 bridge between mission's chassis command and the C-board CDC link."""

import math
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from agv_interfaces.msg import ArmServoCommand, ArmExecution, CBoardOdometry, CBoardStatus, ControlCommand, OdometryFeedback
from agv_interfaces.srv import ArmCommand, JogArm

from .chassis_adapter import ChassisAdapter
from .command_executor import CommandExecutor
from .transport import SerialTransport
from .protocol import CHASSIS_CMD, CHASSIS_STRUCT, STATUS, encode_arm, encode_jog
from .arm_controller import ArmController


class CBoardBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('cboard_bridge_node', **kwargs)
        self._lock = threading.RLock()
        self._serial_port = self._param('serial_port', '/dev/rm_cboard')
        self._send_rate = float(self._param('send_rate_hz', 50.0))
        self._command_timeout = float(self._param('command_timeout_s', 0.05))
        self._status_timeout = float(self._param('status_timeout_s', 0.2))
        self._odom_timeout = float(self._param('odometry_timeout_s', 0.2))
        self._stop_on_slip = self._param('stop_on_slip', True)
        self._reconnect_interval = float(self._param('reconnect_interval_s', 1.0))
        self._control_topic = self._param('control_topic', '/C/controll')
        self._odom_topic = self._param('odometry_topic', '/C/odometry')
        self._chassis_odom_topic = self._param('chassis_odometry_topic', '/C/chassis_odometry')
        self._status_topic = self._param('status_topic', '/C/status')
        if not all(math.isfinite(v) and v > 0 for v in (
                self._send_rate, self._command_timeout, self._status_timeout,
                self._odom_timeout, self._reconnect_interval)):
            raise ValueError('communication rates and timeouts must be finite and positive')

        self._adapter = ChassisAdapter(
            self._command_timeout, self._status_timeout, self._odom_timeout,
            stop_on_slip=self._stop_on_slip)
        self._arm = ArmController(float(self._param('arm_ack_timeout_s', 2.0)),
                                  float(self._param('arm_action_timeout_s', 30.0)))
        self._tx_sequence = 0
        self._last_tx = None
        self._tx_values = (0., 0., 0., 0)
        self._pending_odometry = None
        self._pending_chassis_odometry = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._command_sub = self.create_subscription(
            ControlCommand, self._control_topic, self._on_command, qos)
        self._odom_pub = self.create_publisher(OdometryFeedback, self._odom_topic, qos)
        self._chassis_odom_pub = self.create_publisher(
            CBoardOdometry, self._chassis_odom_topic, qos)
        self._status_pub = self.create_publisher(CBoardStatus, self._status_topic, qos)
        self._arm_pub = self.create_publisher(ArmExecution, '/C/arm_execution', qos)
        self._arm_service = self.create_service(ArmCommand, '/C/arm_command', self._arm_command)
        self._jog_service = self.create_service(JogArm, '/C/arm_jog', self._arm_jog)
        self._servo_sub = self.create_subscription(ArmServoCommand, '/C/arm_servo', self._arm_servo, qos)
        self._reset_srv = self.create_service(Trigger, '/C/reset_fault', self._reset_fault)
        self._status_timer = self.create_timer(0.05, self._publish_status)
        self._odometry_timer = self.create_timer(0.01, self._publish_odometry)
        self._chassis_odometry_timer = self.create_timer(0.01, self._publish_chassis_odometry)
        self._last_status_log = None
        self._last_arm_log = None
        self._transport = SerialTransport(
            self._serial_port, self._send_rate,
            self._reconnect_interval, self._on_connected, self._on_disconnected,
            self._on_frame, self._get_command, self._log_from_worker, self._on_sent,
            self._get_arm_frames)
        self._transport.start()
        self.get_logger().info(
            f'C-board bridge ready: port={self._serial_port}, '
            f'command={self._control_topic}, odometry={self._odom_topic}, '
            f'input timeout={self._command_timeout}s, send rate={self._send_rate}Hz, '
            f'stop_on_slip={self._stop_on_slip}')

    def _param(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def _log_from_worker(self, level, message):
        # Humble caches severity by call site: do not dynamically switch
        # severities through a single logger invocation.
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warning':
            self.get_logger().warning(message)
        else:
            self.get_logger().error(message)

    def _on_connected(self, now):
        with self._lock:
            self._adapter.on_connected(now)
            self._pending_odometry = None
            self._pending_chassis_odometry = None

    def _on_disconnected(self, now, reason):
        with self._lock:
            was_connected = self._adapter.connected
            self._adapter.on_disconnected(now, reason)
            self._arm.pause_link(now)
            self._pending_odometry = None
            self._pending_chassis_odometry = None
        if was_connected:
            self.get_logger().warning(reason)

    def _on_command(self, sample):
        # Fail closed if another executor drops DDS source-time metadata.
        if not isinstance(sample, tuple) or len(sample) != 2:
            with self._lock:
                self._adapter.clear_command()
            self.get_logger().error('Use CommandExecutor to preserve DDS source timestamps',
                                    throttle_duration_sec=5.)
            return
        message, info = sample
        source_ns = info.get('source_timestamp', 0)
        with self._lock:
            now = time.monotonic()
            age = (time.time_ns() - source_ns) / 1e9 if source_ns > 0 else float('inf')
            accepted = self._adapter.on_command(
                float(message.vx_mps), float(message.vy_mps),
                float(message.omega_deg_s), now, age, source_timestamp_ns=source_ns,
                odom_session_id=message.odom_session_id, motion_epoch=message.motion_epoch)
        if not accepted and any((message.vx_mps, message.vy_mps, message.omega_deg_s)):
            self.get_logger().debug('Ignored chassis command: bridge is not ready')

    def _on_sent(self, frame, now):
        with self._lock:
            if frame[2] != CHASSIS_CMD:
                self._arm.sent(frame, now)
                return
            self._tx_sequence += 1
            self._last_tx = now
            self._tx_values = CHASSIS_STRUCT.unpack(frame[4:-2])

    def _get_command(self, now):
        with self._lock:
            return self._adapter.command_frame(time.monotonic())

    def _get_arm_frames(self, now):
        with self._lock:
            a = self._adapter
            reason = a.health_reason(now)
            if reason in ('serial disconnected', 'STATUS missing or timed out', 'ODOM missing or timed out'):
                self._arm.pause_link(now)
            else:
                self._arm.resume_link(now)
            allowed = a.connected and a.status_fresh(now) and a.status.source == 2
            return self._arm.frames(now, allowed, (a.odom_session_id, a.motion_epoch))

    def _submit_arm(self, request, response, **operation):
        with self._lock:
            a, now = self._adapter, time.monotonic()
            stop = operation.get('cmd') == 0
            binding = (a.odom_session_id, a.motion_epoch)
            recovering = (a.origin_initialized and a.health_reason(now) in (
                'serial disconnected', 'STATUS missing or timed out', 'ODOM missing or timed out') and
                binding == (request.odom_session_id, request.motion_epoch))
            if not stop and not recovering and (not a.status_fresh(now) or a.status.source != 2):
                response.reason = 'C board is not connected in PC mode'
            elif not stop and not recovering and (not a.update(now) or binding !=
                                (request.odom_session_id, request.motion_epoch)):
                response.reason = 'stale arm coordinate/permission binding'
            else:
                try:
                    if recovering:
                        self._arm.pause_link(now)
                    response.accepted, response.reason = self._arm.submit(
                        request.command_id, binding, **operation)
                except ValueError as error:
                    response.reason = str(error)
        return response

    def _arm_command(self, request, response):
        return self._submit_arm(request, response, cmd=int(request.cmd))

    def _arm_jog(self, request, response):
        return self._submit_arm(request, response, ud_rpm=request.ud_rpm, duration=request.duration_s)

    def _arm_servo(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        with self._lock:
            a, now = self._adapter, time.monotonic()
            binding = (a.odom_session_id, a.motion_epoch)
            if (0 <= age <= self._arm.servo_timeout and a.update(now) and
                    binding == (message.odom_session_id, message.motion_epoch)):
                self._arm.servo(message.stream_id, binding, message.yaw_wz, message.fb_rpm,
                                message.enabled, now-age, stamp)

    def _on_frame(self, msg_id, payload, now):
        with self._lock:
            now = time.monotonic()
            previous_odom = self._adapter.odom
            is_new_odom = self._adapter.on_frame(msg_id, payload, now)
            self._arm.ack(msg_id, payload)
            if msg_id == STATUS and self._adapter.status_received == now:
                self._arm.status(self._adapter.status.arm, now)
            if self._adapter.odom is not previous_odom and self._adapter.odom is not None:
                self._pending_chassis_odometry = (self._adapter.odom, now)
            if is_new_odom:
                adapter = self._adapter
                self._pending_odometry = (
                    adapter.local_odom(), adapter.odom, now, adapter.odom_session_id,
                    adapter.motion_epoch, adapter.origin_initialized)
            slip_reported = (self._adapter.odom is not previous_odom
                             and self._adapter.odom is not None
                             and bool(self._adapter.odom.flags & 0x02)
                             and (previous_odom is None or not previous_odom.flags & 0x02))
        if slip_reported and not self._stop_on_slip:
            self.get_logger().warning(
                'C-board reports ODOM slip; continuing under stop_on_slip=false '
                '(raw odom_status is preserved)', throttle_duration_sec=5.)

    def _publish_chassis_odometry(self):
        # Publish only new, fresh samples; never replay stale data on link loss.
        # DDS remains on the ROS executor, never the serial worker.
        with self._lock:
            now = time.monotonic()
            pending = self._pending_chassis_odometry
            self._pending_chassis_odometry = None
            if (pending is None or not self._adapter.connected
                    or not 0 <= now - pending[1] <= self._odom_timeout):
                return
        odom, received = pending
        message = CBoardOdometry()
        stamp_ns = max(0, self.get_clock().now().nanoseconds - int((now - received) * 1e9))
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.board_time_ms = odom.t_ms
        message.odom_status = odom.flags
        message.valid = self._adapter.odom_flags_ok(odom.flags)
        message.x_m = odom.x
        message.y_m = odom.y
        message.yaw_rad = odom.yaw
        message.yaw_deg = math.degrees(odom.yaw)
        message.vx_mps = odom.vx
        message.vy_mps = odom.vy
        message.wz_rad_s = odom.wz
        self._chassis_odom_pub.publish(message)

    def _publish_odometry(self):
        # Capture the session WITH the sample; never mix a pending old sample
        # with a newly reset coordinate frame.
        with self._lock:
            now = time.monotonic()
            pending = self._pending_odometry
            self._pending_odometry = None
            if (pending is None or not self._adapter.connected
                    or now-pending[2] > self._odom_timeout
                    or pending[3] != self._adapter.odom_session_id):
                return
        local, raw, received, session, epoch, initialized = pending
        message = OdometryFeedback()
        stamp_ns = max(0, self.get_clock().now().nanoseconds-int((now-received)*1e9))
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = 'c_odom_' + session
        message.odom_session_id = session
        message.motion_epoch = epoch
        message.origin_initialized = initialized
        message.board_time_ms = raw.t_ms
        message.odom_status = raw.flags
        message.valid = initialized and self._adapter.odom_flags_ok(raw.flags)
        if local is not None:
            message.x_m, message.y_m = local.x, local.y
            message.yaw_rad = local.yaw
            message.yaw_deg = math.degrees(local.yaw)
        message.vx_mps, message.vy_mps, message.wz_rad_s = raw.vx, raw.vy, raw.wz
        self._odom_pub.publish(message)

    def _status_message(self):
        with self._lock:
            # Capture time in the same critical section as feedback timestamps;
            # otherwise a concurrent RX can appear to come from the future.
            now = time.monotonic()
            adapter = self._adapter
            status = adapter.status
            odom = adapter.odom
            msg = CBoardStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.connected = adapter.connected
            msg.status_valid = adapter.status_fresh(now)
            msg.odometry_valid = adapter.odom_fresh(now) and odom is not None and adapter.odom_flags_ok(odom.flags) and adapter.origin_initialized
            msg.pc_control = bool(status is not None and adapter.status_fresh(now) and status.source == 2)
            msg.motion_allowed = bool(adapter.update(now))
            if status is not None:
                msg.chassis_status = status.chassis
                msg.arm_state = status.arm
                msg.online_mask = status.online
                msg.error_flags = status.errors
                msg.fb_pulses = status.fb_pulses
                msg.ud_pulses = status.ud_pulses
                msg.pos_flags = status.pos_flags
                msg.yaw_rad = status.yaw_rad
                msg.control_source = status.source
                msg.board_time_ms = status.t_ms
            if odom is not None:
                msg.odom_status = odom.flags
            msg.fault_reason = adapter.health_reason(now)
            msg.odom_session_id = adapter.odom_session_id
            msg.motion_epoch = adapter.motion_epoch
            msg.origin_initialized = adapter.origin_initialized
            msg.command_fresh = adapter.command_fresh(now)
            msg.command_age_s = adapter.command_age(now)
            msg.command_reason = adapter.command_reason
            msg.tx_sequence = self._tx_sequence
            msg.tx_age_s = -1. if self._last_tx is None else max(0., now-self._last_tx)
            msg.sent_vx_mps, msg.sent_vy_mps, msg.sent_wz_rad_s, msg.sent_mode = self._tx_values
            return msg

    def _publish_status(self):
        message = self._status_message()
        self._status_pub.publish(message)
        with self._lock:
            arm = ArmExecution(command_id=self._arm.command_id, state=self._arm.state,
                               pending=self._arm.pending or self._arm.servo_busy,
                               idle=message.status_valid and message.pc_control and self._arm.idle and not self._arm.servo_busy,
                               reason=self._arm.reason)
            arm.header.stamp = self.get_clock().now().to_msg()
        self._arm_pub.publish(arm)
        arm_log = (arm.command_id, arm.state, arm.reason)
        if arm.command_id and arm_log != self._last_arm_log:
            self._last_arm_log = arm_log
            self.get_logger().info(f'ARM {arm.command_id} {arm.state} pending={arm.pending} {arm.reason}')
        if message.fault_reason != self._last_status_log:
            if message.fault_reason:
                self.get_logger().warning(f'Motion inhibited: {message.fault_reason}')
            else:
                self.get_logger().info('C-board feedback healthy; PC control available')
            self._last_status_log = message.fault_reason

    def _reset_fault(self, request, response):
        del request
        with self._lock:
            ok, reason = self._adapter.reset_fault(time.monotonic())
        response.success = ok
        response.message = reason
        return response

    def destroy_node(self):
        if hasattr(self, '_transport'):
            with self._lock:
                frames = (encode_jog(), encode_arm(0)) if self._arm.pending or self._arm.servo_busy else ()
            self._transport.stop(frames)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CBoardBridgeNode()
    executor = CommandExecutor(node._command_sub)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

