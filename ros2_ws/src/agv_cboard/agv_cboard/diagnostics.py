"""Read-only capture, timed velocity, or odometry-based relative motion."""
import argparse
from contextlib import nullcontext
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from agv_interfaces.msg import CBoardOdometry, CBoardStatus, ControlCommand, OdometryFeedback
from .relative_move import RelativeMove


class Diagnostics(Node):
    def __init__(self, writer=None):
        super().__init__('cboard_diagnostics')
        self.writer = writer
        self.start = time.monotonic()
        self.status = None
        self.status_at = None
        self.raw_samples = []
        self.odom = None
        self.odom_at = None
        self.subscriptions_ = [
            self.create_subscription(CBoardOdometry, '/C/chassis_odometry',
                                     lambda m: self.record('raw', m), 10),
            self.create_subscription(OdometryFeedback, '/C/odometry',
                                     self.on_odometry, 10),
            self.create_subscription(CBoardStatus, '/C/status', self.on_status, 10)]
        self.pub = None

    def record(self, kind, message):
        now = time.monotonic()
        if self.writer is not None:
            self.writer.writerow((f'{now-self.start:.9f}', kind,
                                  json.dumps(message_to_ordereddict(message), ensure_ascii=False)))
        if kind == 'raw':
            self.raw_samples.append((now, message.yaw_rad, message.wz_rad_s))

    def on_status(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= .2:
            return
        self.status, self.status_at = message, time.monotonic()-age
        self.record('status', message)

    def on_odometry(self, message):
        self.record('local', message)
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= .2:
            return
        self.odom, self.odom_at = message, time.monotonic()-age

    def odometry_ready(self):
        return bool(self.odom is not None and self.odom_at is not None
                    and time.monotonic()-self.odom_at <= .2
                    and self.odom.valid and self.odom.origin_initialized
                    and self.status is not None
                    and (self.odom.odom_session_id, self.odom.motion_epoch) ==
                    (self.status.odom_session_id, self.status.motion_epoch)
                    and all(math.isfinite(v) for v in self.pose()))

    def pose(self):
        return (self.odom.x_m, self.odom.y_m, self.odom.yaw_rad)

    def ready(self):
        return bool(self.status is not None and self.status_at is not None
                    and time.monotonic()-self.status_at <= .2 and self.status.motion_allowed)

    def wait_for_control(self, args, require_odometry=False):
        self.pub = self.create_publisher(ControlCommand, '/C/controll', 1)
        deadline = time.monotonic()+args.wait
        # Allow graph discovery before deciding that this is the sole publisher.
        discovery = time.monotonic()+1.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if time.monotonic() < discovery:
                continue
            pubs = self.get_publishers_info_by_topic('/C/controll')
            if any(p.node_name != self.get_name() for p in pubs) or len(pubs) > 1:
                raise RuntimeError('Other /C/controll publisher detected; stop mission/debug/CLI first')
            if (self.ready() and self.pub.get_subscription_count() > 0 and len(pubs) == 1
                    and (not require_odometry or self.odometry_ready())):
                break
        else:
            raise RuntimeError('Timed out waiting for PC/healthy feedback and matching subscriber')
        return self.status.odom_session_id, self.status.motion_epoch

    def check_control(self, binding):
        if (not self.ready() or binding != (self.status.odom_session_id, self.status.motion_epoch)):
            raise RuntimeError('Feedback/PC/session changed; motion aborted')
        pubs = self.get_publishers_info_by_topic('/C/controll')
        if len(pubs) != 1 or pubs[0].node_name != self.get_name():
            raise RuntimeError('Competing command publisher appeared; motion aborted')

    def stop_motion(self, duration=.15):
        zero = ControlCommand()
        stop_until = time.monotonic()+duration
        while rclpy.ok() and time.monotonic() < stop_until:
            self.pub.publish(zero)
            self.record('published_command', zero)
            rclpy.spin_once(self, timeout_sec=.02)

    def pulse(self, args):
        binding = self.wait_for_control(args)
        command = ControlCommand(vx_mps=args.vx, vy_mps=args.vy, omega_deg_s=args.omega,
                                 odom_session_id=binding[0], motion_epoch=binding[1])
        started = None
        next_send = time.monotonic()
        sent = 0
        self.get_logger().info('Matched and healthy; pulse timing starts at first nonzero publish')
        try:
            while rclpy.ok() and (started is None or time.monotonic()-started < args.duration):
                now = time.monotonic()
                self.check_control(binding)
                if now >= next_send:
                    self.pub.publish(command)
                    if started is None:
                        started = time.monotonic()
                    self.record('published_command', command)
                    sent += 1
                    next_send = time.monotonic()+.02
                rclpy.spin_once(self, timeout_sec=min(.005,max(0.,next_send-time.monotonic())))
        finally:
            elapsed = 0. if started is None else time.monotonic()-started
            self.stop_motion()
            self.get_logger().info(f'Pulse ended: {sent} nonzero messages; elapsed={elapsed:.3f}s')

    def move(self, args):
        binding = self.wait_for_control(args, require_odometry=True)
        motion = args.motion
        origin = self.pose()
        started = next_send = last_progress = time.monotonic()
        best_progress = 0.
        unit = 'deg' if motion.axis == 'yaw' else 'm'
        self.get_logger().info(
            f'Relative move: axis={motion.axis}, target={motion.target:g} {unit}; '
            'reference is the body orientation at this start')
        try:
            while rclpy.ok():
                now = time.monotonic()
                self.check_control(binding)
                if not self.odometry_ready():
                    raise RuntimeError('Odometry stale/invalid/session changed; motion aborted')
                displacement = motion.displacement(origin, self.pose())
                remaining = motion.remaining(displacement)
                if remaining <= motion.tolerance:
                    self.get_logger().info(
                        f'Target threshold reached: displacement={displacement:.5f} {unit}')
                    break
                if now-started >= args.timeout:
                    raise RuntimeError('Motion timeout; target was NOT reached')
                progress = abs(motion.target)-remaining
                quantum = min(motion.tolerance*.5, abs(motion.target)*.01)
                if progress > best_progress+quantum:
                    best_progress, last_progress = progress, now
                if now-last_progress >= args.stall_timeout:
                    raise RuntimeError('No progress along target direction; motion aborted')
                if now >= next_send:
                    vx, vy, omega = motion.command(displacement)
                    command = ControlCommand(
                        vx_mps=vx, vy_mps=vy, omega_deg_s=omega,
                        odom_session_id=binding[0], motion_epoch=binding[1])
                    self.pub.publish(command)
                    self.record('published_command', command)
                    next_send = time.monotonic()+.02
                rclpy.spin_once(self, timeout_sec=min(.005, max(0., next_send-time.monotonic())))
        finally:
            self.stop_motion(.5)
            if self.odometry_ready() and binding == (self.status.odom_session_id, self.status.motion_epoch):
                final = self.pose()
                displacement = motion.displacement(origin, final)
                self.get_logger().info(
                    f'After 0.5s zero command: measured={displacement:.5f} {unit}, '
                    f'target={motion.target:.5f} {unit}, error={displacement-motion.target:+.5f} {unit}; '
                    f'dx={final[0]-origin[0]:+.5f} m, dy={final[1]-origin[1]:+.5f} m, '
                    f'dyaw={math.degrees(final[2]-origin[2]):+.3f} deg. '
                    'This observation does not guarantee physical standstill.')

    def report_drift(self):
        values = self.raw_samples
        if len(values) < 2:
            self.get_logger().warning('Not enough raw ODOM samples to estimate yaw drift')
            return
        ts = [v[0]-values[0][0] for v in values]
        center = sum(ts)/len(ts)
        mean_yaw = sum(v[1] for v in values)/len(values)
        denominator = sum((t-center)**2 for t in ts)
        slope = sum((t-center)*(v[1]-mean_yaw) for t,v in zip(ts,values))/denominator if denominator else 0.
        mean_wz = sum(v[2] for v in values)/len(values)
        integral = sum((b[0]-a[0])*(b[2]+a[2])*.5 for a,b in zip(values,values[1:]))
        self.get_logger().info(f'RAW yaw slope={math.degrees(slope)*60:.4f} deg/min; '
                               f'mean wz={mean_wz:.6f} rad/s; integral={math.degrees(integral):.4f} deg. '
                               'A stationary vehicle is required to interpret this as drift; inspect session/status events.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('monitor','pulse','move'))
    parser.add_argument('--output', help='optional CSV output path; default: no recording')
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--wait', type=float, default=15.)
    parser.add_argument('--vx', type=float, default=0., help='body FORWARD m/s (default: 0)')
    parser.add_argument('--vy', type=float, default=0., help='body LEFT m/s (default: 0)')
    parser.add_argument('--omega', type=float, default=0., help='CCW deg/s')
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument('--dx', type=float, help='relative FORWARD displacement m; negative = backward')
    targets.add_argument('--dy', type=float, help='relative LEFT displacement m; negative = right')
    targets.add_argument('--angle', type=float, help='relative CCW angle deg; negative = clockwise')
    parser.add_argument('--speed', type=float, default=.05, help='move translation maximum m/s')
    parser.add_argument('--tolerance-m', type=float, default=.005, help='translation stop threshold m')
    parser.add_argument('--tolerance-deg', type=float, default=1., help='rotation stop threshold deg')
    parser.add_argument('--timeout', type=float, default=60., help='move maximum running time s')
    parser.add_argument('--stall-timeout', type=float, default=3., help='move no-progress timeout s')
    args = parser.parse_args(argv)
    explicit_duration = args.duration is not None
    args.duration = args.duration if args.duration is not None else (60. if args.mode == 'monitor' else 3.)
    if not all(math.isfinite(v) for v in (args.duration,args.wait,args.vx,args.vy,args.omega)) or min(args.duration,args.wait) <= 0:
        parser.error('durations must be positive and all values finite')
    if args.mode == 'move':
        if explicit_duration or args.vx or args.vy:
            parser.error('move uses --timeout/--speed and a target, not --duration/--vx/--vy')
        if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.stall_timeout)):
            parser.error('move timeouts must be finite and positive')
        if args.angle is not None:
            axis, target, speed, tolerance = 'yaw', args.angle, args.omega or 10., args.tolerance_deg
        elif args.dx is not None or args.dy is not None:
            if args.omega:
                parser.error('translation move uses --speed, not --omega')
            axis = 'x' if args.dx is not None else 'y'
            target = args.dx if args.dx is not None else args.dy
            speed, tolerance = args.speed, args.tolerance_m
        else:
            parser.error('move requires exactly one of --dx/--dy/--angle')
        try:
            args.motion = RelativeMove(axis, target, speed, tolerance)
        except ValueError as exc:
            parser.error(str(exc))
    elif any(v is not None for v in (args.dx, args.dy, args.angle)):
        parser.error('--dx/--dy/--angle require move mode')
    rclpy.init()
    node = None
    try:
        output = Path(args.output).open('x',newline='',encoding='utf-8') if args.output else nullcontext(None)
        with output as stream:
            writer = csv.writer(stream) if stream is not None else None
            if writer is not None:
                writer.writerow(('monotonic_elapsed_s','kind','message_json'))
            node = Diagnostics(writer)
            if args.mode == 'pulse':
                node.pulse(args)
            elif args.mode == 'move':
                node.move(args)
            else:
                deadline = time.monotonic()+args.duration
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                node.report_drift()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
