"""Real ROS nodes/services with a PTY screen; no hardware executors or motion."""

import os
import pty
import select
import time
from types import SimpleNamespace
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agv_interfaces.action import ScanRoads
from agv_interfaces.msg import CBoardStatus, Edge, ExecutorRuntime, MissionState, MissionTaskInfo, WorkStepRecord
from agv_interfaces.srv import ConfigureMission
from agv_mission.context import ActiveTask
from agv_mission.mission_node import MissionNode
from agv_mission.screen_config import parse_screen_config
from agv_mission.stages import Stage, expected_steps
from agv_planning.planner import Topology
from agv_screen.protocol import END, ScreenMessageParser
from agv_screen.screen_node import ScreenNode


class ScreenIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.mission = MissionNode(parameter_overrides=[Parameter('screen_controlled', value=True)],
                                   cli_args=['--ros-args', '-r', '/C/controll:=/screen_test/control'])
        # Drive mission ticks explicitly so the suite never starts task executors.
        for timer in (self.mission.timer, self.mission.state_timer, self.mission.control_timer):
            timer.cancel()
        self.screen = ScreenNode(parameter_overrides=[
            Parameter('serial_port', value=os.ttyname(self.slave))])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mission)
        self.executor.add_node(self.screen)
        self.received = b''

    def tearDown(self):
        self.executor.shutdown()
        self.screen.destroy_node()
        self.mission.destroy_node()
        os.close(self.master)
        os.close(self.slave)

    def spin_until(self, condition, timeout=3):
        deadline = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), deadline, '等待 ROS2 消息/状态超时')
            self.executor.spin_once(timeout_sec=0.02)

    def expect_command(self, text):
        target = text.encode('ascii') + END

        def received():
            if select.select([self.master], [], [], 0)[0]:
                self.received += os.read(self.master, 8192)
            return target in self.received

        self.spin_until(received)
        end = self.received.index(target) + len(target)
        self.received = self.received[end:]

    def configure(self, start=3, stop=1, routes=(1, 5)):
        return self.mission._configure_and_start(ConfigureMission.Request(
            expected_run_id=self.mission.ctx.run_id, start_zone=start, stop_zone=stop,
            forbidden_routes=list(routes)), ConfigureMission.Response())

    def ready_snapshots(self):
        m = self.mission
        for key in ('route', 'lidar', 'work'):
            m.runtime[key] = (ExecutorRuntime(
                run_id=m.ctx.run_id, executor_session_id=key + '-test', source='REAL',
                quiescent=True, motion_released=True, travel_ready=True, config_ready=True,
                home_node=m.home, current_node=m.home, position_known=True,
                current_heading=m.initial_heading, heading_known=True, odometry_ready=True), time.monotonic())
            m.sessions[key] = key + '-test'
        m.reset_done.update(('route', 'lidar', 'work'))
        board = CBoardStatus(odom_session_id='test-odom', motion_epoch=1,
                            motion_allowed=True, origin_initialized=True)
        board.header.stamp = m.get_clock().now().to_msg()
        m._on_board(board)

    def test_real_serial_service_and_accepted_qr_display(self):
        self.expect_command('page select_routine')
        os.write(self.master, b'\x88\xff\xff\xffROU')
        os.write(self.master, b'TE,1,5,12\r\nCFG,3,1\r\n')
        self.spin_until(lambda: self.screen.zones == (3, 1))
        self.assertEqual(self.mission.ctx.state, 'WAIT_SETUP')
        self.assertIsNone(self.mission._screen_start_at)
        self.assertFalse(self.screen.submitted)
        os.write(self.master, b'START\r\nCFG,1,3\r\nSTART\r\n')
        self.spin_until(lambda: self.mission.ctx.state == 'INITIALIZING')
        self.assertEqual((self.mission.home, self.mission.finish_node), (3, 1))
        self.assertEqual(self.mission.ctx.manual_forbidden_routes, (1, 5, 12))
        self.assertEqual(self.mission.ctx.manual_blocked_edges, ((1, 2), (3, 6), (4, 7)))
        self.assertGreater(self.mission._screen_start_at, time.monotonic())
        self.assertEqual(self.mission.ctx.state, 'INITIALIZING')
        self.spin_until(lambda: self.screen.configure_future is None)

        # Use mission's real QR acceptance path, including task validation and publishing.
        self.mission.ctx.state = 'RUNNING'
        self.mission.ctx.stage = Stage.READ_TASK_CODE
        self.mission.ctx.substate = 'WAIT_QR'
        self.mission.ctx.stage_started_at = time.monotonic()
        self.mission._on_qr(String(data='156+123+516+231'))
        self.expect_command('show_number.t0.txt="156+123"')
        self.expect_command('show_number.t1.txt="516+231"')
        self.assertEqual(self.mission.ctx.code.raw, '156+123+516+231')
        self.screen._on_task(MissionTaskInfo(run_id='old-round', valid=True, raw_code='123+321+123+321'))
        self.assertFalse(self.screen.outgoing)
        self.screen._on_state(MissionState(run_id=self.mission.ctx.run_id, state='FINISHED'))
        self.expect_command('page result')
        self.expect_command('t4.txt="0"')
        self.expect_command('t5.txt="0"')

    def test_config_validation_all_zones_and_bidirectional_edges(self):
        for start in (1, 3):
            for stop in (1, 3):
                home, finish, routes, edges = parse_screen_config(start, stop, [1, 5, 1])
                self.assertEqual((home, finish), (1 if start == 1 else 3, 1 if stop == 1 else 3))
                self.assertEqual(routes, (1, 5))
                self.assertEqual(edges, ((1, 2), (4, 7)))
        for start, stop, routes in ((0, 1, []), (1, 2, []), (1, 1, [13])):
            with self.assertRaises(ValueError):
                parse_screen_config(start, stop, routes)
        bad = ConfigureMission.Request(expected_run_id='old-round', start_zone=1, stop_zone=1)
        self.assertFalse(self.mission._configure_and_start(bad, ConfigureMission.Response()).accepted)
        self.assertEqual(self.mission.ctx.state, 'WAIT_SETUP')
        self.assertTrue(self.configure().accepted)
        self.assertFalse(self.configure().accepted)

    def test_countdown_requires_readiness_and_does_not_block(self):
        self.assertTrue(self.configure().accepted)
        result = self.mission._start(Trigger.Request(), Trigger.Response())
        self.assertFalse(result.success)
        self.assertIn('2 秒', result.message)
        self.mission._screen_start_at = time.monotonic() - 1
        result = self.mission._start(Trigger.Request(), Trigger.Response())
        self.assertFalse(result.success)
        self.assertNotEqual(self.mission.ctx.state, 'RUNNING')
        self.mission.init_started = time.monotonic() - self.mission.init_timeout - 1
        self.mission._initialize()
        self.assertEqual(self.mission.ctx.state, 'FAULT')
        self.assertIsNone(self.mission._screen_start_at)

    def test_ready_configuration_and_delayed_start(self):
        self.assertTrue(self.configure(start=3, stop=1).accepted)
        self.ready_snapshots()
        # Readiness discovery is supplied by the test; exercise the actual initialize/start checks.
        self.mission.task_clients.ready = lambda: True
        self.mission._initialize()
        self.assertEqual(self.mission.ctx.state, 'READY')
        self.assertEqual(self.mission.ctx.confirmed_node, 3)
        self.assertEqual(self.mission.initial_heading, 'UP')
        self.mission._tick()
        self.assertEqual(self.mission.ctx.state, 'READY')
        self.mission._screen_start_at = time.monotonic() - 1
        self.mission._tick()
        self.assertEqual(self.mission.ctx.state, 'RUNNING')
        self.assertEqual(self.mission.ctx.stage, Stage.EXIT_START_AREA)
        self.assertIsNone(self.mission._screen_start_at)

    def test_radar_union_is_used_by_actual_planner_topology(self):
        self.assertTrue(self.configure(routes=(1, 5)).accepted)
        m = self.mission
        m.ctx.state = 'RUNNING'
        task = ActiveTask('scan-test', 'scan', 'lidar')
        m.ctx.active = task
        m.sessions['lidar'] = 'lidar-test'
        result = ScanRoads.Result(
            run_id=m.ctx.run_id, task_id=task.task_id, executor_session_id='lidar-test',
            source='REAL', outcome='COMPLETED', travel_ready=True, map_revision=1,
            blocked_edges=[Edge(a=2, b=1), Edge(a=6, b=3)])
        m.runtime['route'] = (ExecutorRuntime(), time.monotonic())
        m._action_result(task, SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result))
        self.assertEqual(m.ctx.blocked_edges, ((1, 2), (3, 6), (4, 7)))
        self.assertEqual(m.ctx.substate, 'WAIT_MAP')
        topology = Topology()
        self.assertNotEqual(topology.shortest_path(7, 4, m.ctx.blocked_edges).nodes, (7, 4))
        self.assertNotEqual(topology.shortest_path(4, 7, m.ctx.blocked_edges).nodes, (4, 7))
        # An empty radar snapshot must not remove the manual restrictions.
        result.blocked_edges = []
        m._action_result(task, SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result))
        self.assertEqual(m.ctx.blocked_edges, ((1, 2), (4, 7)))

    def test_round_reset_clears_configuration_and_reopens_screen(self):
        self.expect_command('page select_routine')
        self.assertTrue(self.configure().accepted)
        self.ready_snapshots()
        old_run = self.mission.ctx.run_id
        self.mission.ctx.confirmed_steps[(old_run, 'old-task', 'PICK:1')] = ('PICK', 1, 1, 0)
        self.mission.ctx.state = 'FINISHED'
        response = self.mission._reset(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success, response.message)
        self.assertNotEqual(self.mission.ctx.run_id, old_run)
        self.assertEqual(self.mission.ctx.state, 'WAIT_SETUP')
        self.assertEqual(self.mission.ctx.manual_blocked_edges, ())
        self.assertEqual(self.mission.ctx.confirmed_steps, {})
        self.expect_command('page select_routine')

    def test_controller_counts_are_deduplicated_and_written_after_result_initializes(self):
        self.expect_command('page select_routine')
        m = self.mission
        m.ctx.state = 'RUNNING'
        for batch in (1, 2):
            for operation in ('RAW_PICK', 'COARSE_PROCESS',
                              'STORAGE_PLACE' if batch == 1 else 'STORAGE_STACK'):
                positions = () if operation == 'RAW_PICK' else (1, 2, 3)
                spec = expected_steps(operation, (1, 5, 6), positions)
                task = ActiveTask(f'{batch}-{operation}', 'work', 'work', spec=spec, batch=batch)
                records = [WorkStepRecord(
                    run_id=m.ctx.run_id, task_id=task.task_id, step_id=step_id, batch=batch,
                    operation=op, item_index=index, color=color, position=position,
                    controller_completed=True, visually_verified=False)
                    for step_id, op, index, color, position in spec]
                m._commit_records(task, records[:1])
                m._commit_records(task, records)
                m._commit_records(task, records)
        self.assertEqual(len(m.ctx.confirmed_steps), 24)
        m._publish()
        self.spin_until(lambda: self.screen.state.pick_count == 12)
        self.assertEqual(self.screen.state.place_count, 12)
        self.assertFalse(self.screen.outgoing)
        m.ctx.stage = Stage.ENTER_FINISH_AREA
        m._finish()
        self.expect_command('page result')
        page_at = time.monotonic()
        self.expect_command('t4.txt="12"')
        self.assertGreaterEqual(time.monotonic() - page_at, 0.28)
        self.expect_command('t5.txt="12"')
        m._publish()
        self.executor.spin_once(timeout_sec=0.02)
        self.assertFalse(self.screen.outgoing, '重复状态不得再次切换结果页')

    def test_start_requires_both_selection_pages_in_this_round(self):
        self.expect_command('page select_routine')
        os.write(self.master, b'START\r\n')
        self.expect_command('page select_routine')
        os.write(self.master, b'ROUTE\r\nSTART\r\n')
        self.expect_command('page setup')
        self.assertEqual(self.mission.ctx.state, 'WAIT_SETUP')
        self.assertFalse(self.screen.submitted)

    def test_finished_task_snapshot_is_written_before_result_page_copy(self):
        self.expect_command('page select_routine')
        run_id = self.mission.ctx.run_id
        self.screen._on_state(MissionState(run_id=run_id, state='FINISHED', pick_count=12, place_count=12))
        self.screen._on_task(MissionTaskInfo(run_id=run_id, valid=True, raw_code='156+123+516+231'))
        self.assertEqual(list(self.screen.outgoing), [
            'show_number.t0.txt="156+123"', 'show_number.t1.txt="516+231"',
            'page result', 't4.txt="12"', 't5.txt="12"'])
        self.expect_command('show_number.t0.txt="156+123"')
        self.expect_command('show_number.t1.txt="516+231"')
        self.expect_command('page result')
        self.expect_command('t4.txt="12"')
        self.expect_command('t5.txt="12"')

    def test_packaged_parser_rejects_invalid_frames(self):
        parser = ScreenMessageParser()
        self.assertEqual(parser.feed(b'ROUTE,0\r\nROUTE,1,1\r\nCFG,2,1\r\nSTARTED\r\n'), [])
        self.assertEqual(parser.feed(b'ROU'), [])
        self.assertEqual(parser.feed(b'TE\r\nCFG,1,3\r\nSTA'), [('ROUTE', ()), ('CFG', (1, 3))])
        self.assertEqual(parser.feed(b'RT\r\nSTART\r\n'), [('START', ()), ('START', ())])


if __name__ == '__main__':
    unittest.main()
