"""Opt-in ROS/serial integration; opens only a PTY, never real hardware.

AGV_CBOARD_ROS_TEST=1 python3 -m unittest discover -s test -p test_ros_pty.py -v
Run with ROS2 and this workspace sourced, in an isolated ROS_DOMAIN_ID.
"""

import os
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

from agv_cboard.protocol import (
    CHASSIS_CMD, CHASSIS_STRUCT, ODOM, ODOM_STRUCT, STATUS, STATUS_STRUCT,
    FrameParser, encode_frame,
)


class BoardPeer:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.source = 1
        self.x = 0.
        self.y = 0.
        self.yaw = 0.
        self.vx = 0.
        self.vy = 0.
        self.wz = 0.
        self.flags = 0x0D
        self.send_odom = True
        self.simulate_motion = False
        self.command = (0., 0., 0., 0)
        self.frames = []
        self.error = None
        self._closed = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        parser = FrameParser()
        tick = 1
        next_send = time.monotonic()
        last_step = next_send
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                try:
                    data = os.read(self.master, 4096)
                except BlockingIOError:
                    data = b''
                for mid, payload in parser.feed(data):
                    if mid == CHASSIS_CMD:
                        self.frames.append((now, CHASSIS_STRUCT.unpack(payload)))
                        self.command = CHASSIS_STRUCT.unpack(payload)
                if now >= next_send:
                    if self.simulate_motion:
                        dt = now-last_step
                        self.vx, self.vy, self.wz = self.command[:3] if self.command[3] else (0., 0., 0.)
                        c, s = math.cos(self.yaw), math.sin(self.yaw)
                        self.x += (c*self.vx-s*self.vy)*dt
                        self.y += (s*self.vx+c*self.vy)*dt
                        self.yaw += self.wz*dt
                    last_step = now
                    if tick % 5 == 1:
                        os.write(self.master, encode_frame(STATUS, STATUS_STRUCT.pack(
                            tick, 1, 0, 15, 0, 0., 0., 0., self.source)))
                    if self.send_odom:
                        os.write(self.master, encode_frame(ODOM, ODOM_STRUCT.pack(
                            tick, self.x, self.y, self.yaw,
                            self.vx, self.vy, self.wz, self.flags)))
                    tick += 10
                    next_send = now + .01
                self._stop.wait(.001)
        except Exception as exc:
            self.error = exc

    def stop(self):
        if self._closed:
            return
        self._stop.set()
        self._thread.join(1.)
        os.close(self.master)
        os.close(self.slave)
        self._closed = True


@unittest.skipUnless(os.environ.get('AGV_CBOARD_ROS_TEST') == '1',
                     'set AGV_CBOARD_ROS_TEST=1 for isolated ROS + PTY integration')
class RosPtyTests(unittest.TestCase):
    def test_yaw_pipeline_fixed_input_changes_sessions_and_silence(self):
        import rclpy
        from rclpy.parameter import Parameter
        from agv_interfaces.msg import CBoardOdometry, CBoardStatus, OdometryFeedback
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor

        rclpy.init()
        board = BoardPeer()
        board.source = 2
        board.x, board.y, board.yaw = 2., 3., 1.25
        # Deliberately inconsistent feedback: the host must copy yaw, not
        # reconstruct it by integrating velocities or using board/host time.
        board.vx, board.vy, board.wz = .125, -.25, -.5
        bridge = client = executor = None
        try:
            bridge = CBoardBridgeNode(parameter_overrides=[
                Parameter('serial_port', value=board.path),
                Parameter('control_topic', value='/cboard_yaw_test/command'),
                Parameter('odometry_topic', value='/cboard_yaw_test/odom'),
                Parameter('chassis_odometry_topic', value='/cboard_yaw_test/raw'),
                Parameter('status_topic', value='/cboard_yaw_test/status'),
                Parameter('reconnect_interval_s', value=.05),
            ])
            client = rclpy.create_node('cboard_yaw_test_client')
            executor = CommandExecutor(bridge._command_sub)
            executor.add_node(bridge)
            executor.add_node(client)
            raw, local, statuses = [], [], []
            subscriptions = [
                client.create_subscription(CBoardOdometry, '/cboard_yaw_test/raw', raw.append, 10),
                client.create_subscription(OdometryFeedback, '/cboard_yaw_test/odom', local.append, 10),
                client.create_subscription(CBoardStatus, '/cboard_yaw_test/status', statuses.append, 10),
            ]
            board.start()

            def pump(seconds):
                until = time.monotonic()+seconds
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=.005)

            def wait_for(predicate):
                until = time.monotonic()+3.
                while not predicate() and time.monotonic() < until:
                    pump(.01)
                self.assertTrue(predicate(), 'timed out waiting for yaw pipeline')

            wait_for(lambda: local and raw and statuses and statuses[-1].motion_allowed)
            pump(.5)
            self.assertGreater(len(local), 20)
            session = local[-1].odom_session_id
            for message in local:
                self.assertEqual(message.yaw_rad, 0.)
                self.assertEqual(message.yaw_deg, 0.)
                self.assertEqual(message.wz_rad_s, -.5)
            for message in raw:
                self.assertEqual(message.yaw_rad, 1.25)
                self.assertEqual(message.yaw_deg, math.degrees(1.25))

            # Changing position and velocity must not move the yaw origin.
            board.x, board.y = -4., 5.
            board.wz = .75
            wait_for(lambda: local[-1].wz_rad_s == .75)
            self.assertEqual(local[-1].yaw_rad, 0.)
            self.assertEqual(local[-1].odom_session_id, session)

            # Track signed, unwrapped yaw without scaling, filtering or drift.
            for yaw in (-.625, -7.25, 13.):
                board.yaw = yaw
                wait_for(lambda: raw[-1].yaw_rad == yaw and local[-1].yaw_rad == yaw-1.25)
                pump(.1)
                self.assertEqual(local[-1].yaw_deg, math.degrees(yaw-1.25))
                self.assertEqual(local[-1].odom_session_id, session)

            by_tick = {m.board_time_ms: m for m in raw}
            matched = [m for m in local if m.board_time_ms in by_tick]
            self.assertGreater(len(matched), 20)
            for message in matched:
                self.assertEqual(message.yaw_rad, by_tick[message.board_time_ms].yaw_rad-1.25)

            # Changing control source explicitly creates one new origin.
            board.source = 1
            wait_for(lambda: not statuses[-1].pc_control)
            board.yaw = 2.5
            wait_for(lambda: local[-1].yaw_rad == 1.25)
            self.assertEqual(local[-1].odom_session_id, session)
            board.source = 2
            wait_for(lambda: local[-1].odom_session_id != session and local[-1].valid)
            self.assertEqual(local[-1].yaw_rad, 0.)

            # STATUS and TX remain active, with nonzero last reported wz.
            # Neither odometry publisher may repeat or extrapolate old yaw.
            board.send_odom = False
            pump(.3)
            self.assertFalse(statuses[-1].odometry_valid)
            counts = len(raw), len(local)
            pump(.4)
            self.assertEqual((len(raw), len(local)), counts)
            self.assertTrue(board.frames)
            self.assertTrue(all(values == (0., 0., 0., 0) for _, values in board.frames))
            self.assertIsNone(board.error)
            del subscriptions
        finally:
            if bridge is not None:
                executor.remove_node(bridge)
                bridge.destroy_node()
            board.stop()
            if client is not None:
                executor.remove_node(client)
                client.destroy_node()
            if executor is not None:
                executor.shutdown()
            rclpy.shutdown()

    def test_relative_move_cli_targets_and_abort(self):
        board = BoardPeer()
        board.source = 2
        board.simulate_motion = True
        board.x, board.y, board.yaw = 2., 3., math.pi/2
        process = None
        with tempfile.TemporaryDirectory(prefix='cboard_move_') as tmp:
            try:
                board.start()
                process = subprocess.Popen(
                    ['ros2', 'launch', 'agv_cboard', 'cboard.launch.py', f'serial_port:={board.path}'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                cases = (
                    ('forward', ['--dx', '.03', '--speed', '.05'], 0, 1),
                    ('back', ['--dx', '-.03', '--speed', '.05'], 0, -1),
                    ('left', ['--dy', '.03', '--speed', '.05'], 1, 1),
                    ('right', ['--dy', '-.03', '--speed', '.05'], 1, -1),
                    ('ccw', ['--angle', '8', '--omega', '10'], 2, 1),
                    ('cw', ['--angle', '-8', '--omega', '10'], 2, -1),
                )
                for label, arguments, axis, sign in cases:
                    with self.subTest(move=label):
                        start = len(board.frames)
                        before = (board.x, board.y, board.yaw)
                        files_before = set(Path(tmp).iterdir())
                        result = subprocess.run(
                            ['ros2', 'run', 'agv_cboard', 'cboard_diagnostics', 'move',
                             '--timeout', '8', *arguments],
                            cwd=tmp, capture_output=True, text=True, timeout=20)
                        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                        self.assertEqual(set(Path(tmp).iterdir()), files_before)
                        self.assertIn('Target threshold reached', result.stderr+result.stdout)
                        moving = [c for _, c in board.frames[start:] if c[3] == 1]
                        self.assertGreater(len(moving), 10)
                        self.assertTrue(all(c[axis]*sign > 0 for c in moving))
                        self.assertTrue(all(c[i] == 0 for c in moving for i in range(3) if i != axis))
                        dx, dy = board.x-before[0], board.y-before[1]
                        progress = ((math.cos(before[2])*dx+math.sin(before[2])*dy) if axis == 0 else
                                    (-math.sin(before[2])*dx+math.cos(before[2])*dy) if axis == 1 else
                                    math.degrees(board.yaw-before[2]))
                        self.assertAlmostEqual(progress, sign*(.03 if axis < 2 else 8),
                                               delta=.008 if axis < 2 else 1.5)
                        self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
                # Frozen position must not be treated as reaching the target.
                board.simulate_motion = False
                result = subprocess.run(
                    ['ros2', 'run', 'agv_cboard', 'cboard_diagnostics', 'move',
                     '--dx', '.1', '--stall-timeout', '.4', '--timeout', '3',
                     '--output', str(Path(tmp)/'stalled.csv')],
                    capture_output=True, text=True, timeout=15)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('No progress', result.stdout+result.stderr)
                self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
                # Loss of position feedback aborts an active move and sends zero.
                board.simulate_motion = True
                active = subprocess.Popen(
                    ['ros2', 'run', 'agv_cboard', 'cboard_diagnostics', 'move',
                     '--dy', '.3', '--output', str(Path(tmp)/'feedback_loss.csv')],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                try:
                    start = len(board.frames)
                    deadline = time.monotonic()+8
                    while not any(c[3] for _, c in board.frames[start:]) and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertTrue(any(c[3] for _, c in board.frames[start:]))
                    board.send_odom = False
                    output, _ = active.communicate(timeout=5)
                    self.assertNotEqual(active.returncode, 0, output)
                    self.assertIn('aborted', output)
                    self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
                finally:
                    if active.poll() is None:
                        active.kill()
                        active.communicate(timeout=3)
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.communicate(timeout=6)
                board.stop()

    def test_launch_parameters_and_clean_shutdown(self):
        # Launch the installed entry point, exercising main/CommandExecutor
        # and YAML precedence. Both runs use only a software-created PTY.
        for override in (False, True):
            board = BoardPeer()
            board.source = 2
            process = None
            with tempfile.TemporaryDirectory(prefix='cboard_launch_') as tmp:
                config = Path(tmp) / 'cboard.yaml'
                yaml_port = '/tmp/cboard_deliberately_missing' if override else board.path
                config.write_text(
                    'cboard_bridge_node:\n  ros__parameters:\n'
                    f'    serial_port: {yaml_port}\n'
                    '    control_topic: /cboard_launch_test/command\n'
                    '    odometry_topic: /cboard_launch_test/odom\n'
                    '    status_topic: /cboard_launch_test/status\n')
                args = ['ros2', 'launch', 'agv_cboard', 'cboard.launch.py', f'config:={config}']
                if override:
                    args.append(f'serial_port:={board.path}')
                try:
                    board.start()
                    process = subprocess.Popen(
                        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    deadline = time.monotonic() + 6.
                    while not board.frames and process.poll() is None and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertTrue(board.frames, 'installed launch did not open configured PTY')
                    process.send_signal(signal.SIGINT)
                    output, _ = process.communicate(timeout=6.)
                    self.assertEqual(process.returncode, 0, output)
                    self.assertNotIn('Traceback', output)
                    self.assertIn(f'Connected to C board: {board.path}', output)
                    self.assertTrue(all(frame[1][3] == 0 for frame in board.frames))
                finally:
                    if process is not None and process.poll() is None:
                        process.kill()
                        process.communicate(timeout=3.)
                    board.stop()

    def test_connection_timed_pulse_and_readonly_capture(self):
        import csv
        board=BoardPeer()
        board.source=2
        process=None
        with tempfile.TemporaryDirectory(prefix='cboard_diagnostics_') as tmp:
            try:
                board.start()
                process=subprocess.Popen(['ros2','launch','agv_cboard','cboard.launch.py',
                                          f'serial_port:={board.path}'],
                                          stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                output_path=Path(tmp)/'pulse.csv'
                result=subprocess.run(['ros2','run','agv_cboard','cboard_diagnostics','pulse',
                                       '--duration','3','--vx','0.05','--output',str(output_path)],
                                       capture_output=True,text=True,timeout=25)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                moving=[(t,c) for t,c in board.frames if c[3]==1]
                self.assertGreater(len(moving),130)
                self.assertAlmostEqual(moving[-1][0]-moving[0][0],2.98,delta=.12)
                self.assertTrue(all(abs(c[0]-.05)<1e-6 and abs(c[1])<1e-6 for _,c in moving))
                self.assertEqual(board.frames[-1][1][3],0)
                with output_path.open(encoding='utf-8') as stream:
                    rows=list(csv.DictReader(stream))
                self.assertTrue(any(r['kind']=='published_command' for r in rows))
                # Exercise the installed CLI, not just the encoder: omitted
                # axes must remain zero for translation AND rotation.
                for label, arguments, expected in (
                        ('left', ['--vy', '0.05'], (0., .05, 0.)),
                        ('rotate', ['--omega', '10'], (0., 0., 0.1745329252)),
                        ('idle', [], (0., 0., 0.))):
                    with self.subTest(pulse=label):
                        start = len(board.frames)
                        files_before = set(Path(tmp).iterdir())
                        result = subprocess.run(
                            ['ros2', 'run', 'agv_cboard', 'cboard_diagnostics', 'pulse',
                             '--duration', '.4', *arguments],
                            cwd=tmp, capture_output=True, text=True, timeout=20)
                        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                        self.assertEqual(set(Path(tmp).iterdir()), files_before)
                        commands = [c for _, c in board.frames[start:] if c[3] == 1]
                        if any(expected):
                            self.assertGreater(len(commands), 10)
                            for command in commands:
                                for actual, target in zip(command[:3], expected):
                                    self.assertAlmostEqual(actual, target, places=6)
                        else:
                            self.assertFalse(commands)
                        self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
                count=len([c for _,c in board.frames if c[3]==1])
                result=subprocess.run(['ros2','run','agv_cboard','cboard_diagnostics','monitor',
                                       '--duration','1'], cwd=tmp,
                                       capture_output=True,text=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertEqual(len([c for _,c in board.frames if c[3]==1]),count)
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.communicate(timeout=6)
                board.stop()

    def test_real_ros_and_serial_safety_path(self):
        self._real_ros_and_serial_safety_path(stop_on_slip=True)

    def test_real_ros_and_serial_slip_warning_policy(self):
        self._real_ros_and_serial_safety_path(stop_on_slip=False)

    def _real_ros_and_serial_safety_path(self, stop_on_slip):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile
        from std_srvs.srv import Trigger
        from agv_interfaces.msg import (
            CBoardOdometry, CBoardStatus, ControlCommand, OdometryFeedback,
        )
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor

        rclpy.init()
        board = BoardPeer()
        temp = tempfile.TemporaryDirectory(prefix='cboard_test_')
        path = os.path.join(temp.name, 'board')
        os.symlink(board.path, path)
        executor = None
        bridge = client = None
        try:
            bridge = CBoardBridgeNode(parameter_overrides=[
                Parameter('serial_port', value=path),
                Parameter('control_topic', value='/cboard_test/command'),
                Parameter('odometry_topic', value='/cboard_test/odom'),
                Parameter('chassis_odometry_topic', value='/cboard_test/chassis_odom'),
                Parameter('status_topic', value='/cboard_test/status'),
                Parameter('reconnect_interval_s', value=.05),
                Parameter('stop_on_slip', value=stop_on_slip),
            ])
            client = rclpy.create_node('cboard_pty_test_client')
            executor = CommandExecutor(bridge._command_sub)
            pub = client.create_publisher(
                ControlCommand, '/cboard_test/command',
                QoSProfile(depth=1))
            statuses, odoms, chassis_odoms = [], [], []
            sub1 = client.create_subscription(
                CBoardStatus, '/cboard_test/status', statuses.append, 10)
            sub2 = client.create_subscription(
                OdometryFeedback, '/cboard_test/odom', odoms.append, 10)
            sub3 = client.create_subscription(
                CBoardOdometry, '/cboard_test/chassis_odom', chassis_odoms.append, 10)
            reset = client.create_client(Trigger, '/C/reset_fault')
            executor.add_node(bridge)
            executor.add_node(client)
            board.start()

            def pump(seconds, command=None):
                deadline = time.monotonic() + seconds
                next_pub = 0.
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if command is not None and now >= next_pub:
                        pub.publish(ControlCommand(
                            vx_mps=command[0], vy_mps=command[1], omega_deg_s=command[2]))
                        next_pub = now + .02
                    executor.spin_once(timeout_sec=.005)

            def rearm():
                pump(.15, (0., 0., 0.))
                future = reset.call_async(Trigger.Request())
                deadline = time.monotonic() + 2.
                while not future.done() and time.monotonic() < deadline:
                    pump(.01, (0., 0., 0.))
                self.assertTrue(future.done())
                self.assertTrue(future.result().success, future.result().message)

            # Non-PC command stream must never leak into the wire.
            pump(.35, (.2, .3, 90.))
            self.assertTrue(board.frames)
            self.assertTrue(all(frame[1][3] == 0 for frame in board.frames))
            self.assertTrue(chassis_odoms, 'RC feedback must be observable at startup')
            self.assertTrue(odoms, 'new RC observations must be published')
            self.assertFalse(odoms[-1].origin_initialized)
            intervals = [b[0] - a[0] for a, b in zip(board.frames, board.frames[1:])]
            self.assertGreater(len(intervals), 8)
            self.assertLess(sum(intervals) / len(intervals), .03)
            self.assertGreater(sum(intervals) / len(intervals), .015)
            board.source = 2
            pump(.2, (0., 0., 0.))
            self.assertTrue(statuses[-1].pc_control)

            # Waiting for the state lock must not turn a newer RX timestamp
            # into an apparent negative age and latch a false timeout.
            snapshots = []
            started = threading.Event()

            def snapshot():
                started.set()
                snapshots.append(bridge._status_message())

            with bridge._lock:
                reader = threading.Thread(target=snapshot)
                reader.start()
                self.assertTrue(started.wait(1.))
                time.sleep(.02)
                bridge._adapter.status_received = time.monotonic()
                bridge._adapter.odom_received = time.monotonic()
            reader.join(1.)
            self.assertEqual(len(snapshots), 1)
            self.assertTrue(snapshots[0].motion_allowed, snapshots[0].fault_reason)
            pump(.25, (.2, .3, 90.))
            moving = [frame for _, frame in board.frames if frame[3] == 1]
            self.assertTrue(moving)
            self.assertAlmostEqual(moving[-1][0], .2, places=6)
            self.assertAlmostEqual(moving[-1][1], .3, places=6)
            self.assertAlmostEqual(moving[-1][2], 1.5707963, places=6)
            self.assertTrue(odoms)

            # Receive watchdog zeros on ROS silence while hardware stays healthy.
            pump(.3)
            self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
            self.assertTrue(statuses[-1].motion_allowed)
            self.assertGreater(statuses[-1].tx_sequence,0)
            self.assertEqual(statuses[-1].sent_mode,0)
            self.assertLess(statuses[-1].tx_age_s,.1)

            # Source-time validation discards a queued, expired nonzero command.
            pub.publish(ControlCommand(vx_mps=1.))
            time.sleep(.25)
            start = len(board.frames)
            pump(.1)
            self.assertTrue(all(frame[1][3] == 0 for frame in board.frames[start:]))

            # A blocked DDS publisher must not block CDC heartbeats/watchdogs.
            original_publish = bridge._odom_pub.publish
            blocked = threading.Event()

            def slow_publish(message):
                if not blocked.is_set():
                    blocked.set()
                    time.sleep(.3)
                original_publish(message)

            pump(.1, (.2, 0., 0.))
            start = len(board.frames)
            bridge._odom_pub.publish = slow_publish
            try:
                pump(.4)
            finally:
                bridge._odom_pub.publish = original_publish
            samples = board.frames[start:]
            self.assertTrue(blocked.is_set())
            self.assertGreater(len(samples), 10)
            self.assertLess(max(b[0] - a[0] for a, b in zip(samples, samples[1:])), .15)
            self.assertEqual(samples[-1][1][3], 0)
            self.assertTrue(statuses[-1].motion_allowed)

            # Position follows signed XY; returning to start returns to zero, yaw stays unwrapped.
            board.y = 1.
            board.yaw = 4 * 3.141592653589793
            pump(.1, (0., 0., 0.))
            board.y = 0.
            pump(.1, (0., 0., 0.))
            self.assertAlmostEqual(odoms[-1].y_m, 0., places=3)
            self.assertAlmostEqual(odoms[-1].yaw_deg, 720., places=3)

            # RC takeover stops PC motion while both observation topics continue.
            pump(.1, (.2, 0., 0.))
            board.source = 1
            pump(.15, (.2, 0., 0.))
            count = len(odoms)
            raw_count = len(chassis_odoms)
            board.x = .25
            board.y = .75
            board.yaw = -.5
            board.vx, board.vy, board.wz = .1, .2, -.3
            pump(.1, (.2, 0., 0.))
            self.assertGreater(len(odoms), count)
            self.assertGreater(len(chassis_odoms), raw_count)
            raw = chassis_odoms[-1]
            self.assertTrue(raw.valid)
            self.assertEqual(raw.odom_status, 13)
            self.assertGreater(raw.board_time_ms, 0)
            for value, expected in ((raw.x_m, .25), (raw.y_m, .75),
                                    (raw.yaw_rad, -.5), (raw.yaw_deg, -28.6478898),
                                    (raw.vx_mps, .1), (raw.vy_mps, .2),
                                    (raw.wz_rad_s, -.3)):
                self.assertAlmostEqual(value, expected, places=5)
            self.assertEqual(board.frames[-1][1][3], 0)
            board.source = 2
            pump(.15, (.2, 0., 0.))
            self.assertTrue(statuses[-1].motion_allowed)
            self.assertGreater(len(odoms), count)
            self.assertEqual(board.frames[-1][1][3], 1)
            self.assertAlmostEqual(odoms[-1].x_m, 0.)
            self.assertAlmostEqual(odoms[-1].y_m, 0.)
            self.assertAlmostEqual(odoms[-1].yaw_rad, 0.)
            self.assertGreater(chassis_odoms[-1].board_time_ms, raw.board_time_ms)
            rearm()
            pump(.12, (.2, 0., 0.))
            self.assertEqual(board.frames[-1][1][3], 1)

            # ODOM silence inhibits even though STATUS is still arriving.
            board.send_odom = False
            pump(.3, (.2, 0., 0.))
            self.assertEqual(board.frames[-1][1][3], 0)
            self.assertFalse(statuses[-1].odometry_valid)
            raw_count = len(chassis_odoms)
            pump(.1)
            self.assertEqual(len(chassis_odoms), raw_count, 'do not replay stale ODOM')
            board.send_odom = True
            pump(.1, (0., 0., 0.))
            rearm()

            # Preserve raw flags; all three validity outputs follow the policy.
            session, epoch = statuses[-1].odom_session_id, statuses[-1].motion_epoch
            board.flags = 15
            pump(.15, (0., 2., 0.))
            self.assertEqual(chassis_odoms[-1].valid, not stop_on_slip)
            self.assertEqual(odoms[-1].valid, not stop_on_slip)
            self.assertEqual(statuses[-1].odometry_valid, not stop_on_slip)
            self.assertEqual(chassis_odoms[-1].odom_status, 15)
            self.assertEqual(odoms[-1].odom_status, 15)
            self.assertEqual(statuses[-1].odom_status, 15)
            self.assertEqual(statuses[-1].motion_allowed, not stop_on_slip)
            if stop_on_slip:
                self.assertIn('status=0x0F', statuses[-1].fault_reason)
                self.assertEqual(board.frames[-1][1][3], 0)
                self.assertGreater(statuses[-1].motion_epoch, epoch)
            else:
                self.assertEqual(statuses[-1].fault_reason, '')
                self.assertEqual((statuses[-1].odom_session_id, statuses[-1].motion_epoch),
                                 (session, epoch))
                self.assertEqual(board.frames[-1][1], (0., 2., 0., 1))
            board.flags = 13
            pump(.15, (0., 0., 0.))
            self.assertTrue(chassis_odoms[-1].valid)
            self.assertTrue(statuses[-1].odometry_valid)
            self.assertTrue(statuses[-1].motion_allowed)
            self.assertEqual(statuses[-1].fault_reason, '')

            # The software-only device really disappears, then re-enumerates.
            board.stop()
            pump(.12, (0., 0., 0.))
            self.assertFalse(statuses[-1].connected)
            board = BoardPeer()
            board.source = 2
            os.unlink(path)
            os.symlink(board.path, path)
            board.start()
            pump(.25, (.2, 0., 0.))
            self.assertTrue(statuses[-1].connected)
            self.assertTrue(statuses[-1].motion_allowed)
            self.assertTrue(any(frame[1][3] == 1 for frame in board.frames))
            rearm()
            pump(.12, (.2, 0., 0.))
            self.assertEqual(board.frames[-1][1][3], 1)

            # Clean shutdown sends a final zero frame before closing the port.
            executor.remove_node(bridge)
            bridge.destroy_node()
            bridge = None
            pump(.05)
            self.assertEqual(board.frames[-1][1], (0., 0., 0., 0))
            self.assertIsNone(board.error)
            del sub1, sub2, sub3
        finally:
            if bridge is not None:
                executor.remove_node(bridge)
                bridge.destroy_node()
            board.stop()
            if client is not None:
                executor.remove_node(client)
                client.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            temp.cleanup()


if __name__ == '__main__':
    unittest.main()
