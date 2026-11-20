"""Exercise disabled-lidar initialization, map acknowledgement, stop and reset without hardware."""

from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from agv_interfaces.msg import CBoardStatus, Edge, ExecutorRuntime
from agv_interfaces.srv import ConfigureMission
from agv_mission.mission_node import MissionNode
from agv_mission.stages import Stage


class LidarPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.m = MissionNode(parameter_overrides=[Parameter('use_lidar', value=False),
                                                  Parameter('screen_controlled', value=True)],
                             cli_args=['--ros-args', '-r', '/C/controll:=/lidar_policy_test/control'])
        for timer in self.m.timers:
            timer.cancel()

    def tearDown(self):
        self.m.destroy_node()

    def snapshots(self):
        for key in ('route', 'work'):
            self.m.runtime[key] = (ExecutorRuntime(
                run_id=self.m.ctx.run_id, source='REAL', executor_session_id=key + '-policy',
                quiescent=True, motion_released=True, travel_ready=True, config_ready=True,
                position_known=True, current_node=self.m.home, current_heading=self.m.initial_heading,
                heading_known=True, odometry_ready=True), time.monotonic())
            self.m.sessions[key] = key + '-policy'
        board = CBoardStatus(odom_session_id='policy-test', motion_epoch=1,
                            motion_allowed=True, origin_initialized=True)
        board.header.stamp = self.m.get_clock().now().to_msg()
        self.m._on_board(board)

    def configure_ready(self, routes=(1, 5)):
        result = self.m._configure_and_start(ConfigureMission.Request(
            expected_run_id=self.m.ctx.run_id, start_zone=3, stop_zone=1,
            forbidden_routes=list(routes)), ConfigureMission.Response())
        self.assertTrue(result.accepted, result.reason)
        self.snapshots()
        self.m.reset_done.update(('route', 'work'))
        self.m.task_clients.ready = Mock(return_value=True)
        self.m._initialize()
        self.assertEqual(self.m.ctx.state, 'READY')

    def scan_stage(self):
        self.configure_ready()
        self.m.ctx.state = 'RUNNING'
        self.m.ctx.stage = Stage.STARTUP_SCAN
        self.m._enter_stage()

    def test_no_lidar_clients_or_runtime_needed_and_screen_delay_is_preserved(self):
        self.assertIsNone(self.m.task_clients.scan)
        self.assertEqual(set(self.m.task_clients.get), {'route', 'work'})
        self.assertEqual(set(self.m.task_clients.reset), {'route', 'work'})
        self.assertEqual(set(self.m.task_clients.stop), {'route', 'work'})
        self.configure_ready()
        response = self.m._start(Trigger.Request(), Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('2 秒', response.message)
        self.m._screen_start_at = time.monotonic() - 1
        response = self.m._start(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.m.ctx.stage, Stage.EXIT_START_AREA)

    def test_manual_map_must_be_acknowledged_before_next_route(self):
        self.scan_stage()
        c = self.m.ctx
        self.assertEqual(c.substate, 'WAIT_MAP')
        self.assertIsNone(c.active)  # There is no fabricated scan Action result.
        self.assertEqual(c.blocked_edges, ((1, 2), (4, 7)))
        self.assertEqual(c.map_revision, 1)
        rt = self.m.runtime['route'][0]
        rt.map_ready, rt.map_revision = True, 1
        rt.blocked_edges = [Edge(a=1, b=2)]  # Missing one manually forbidden road.
        self.m._commit_map()
        self.assertEqual(c.stage, Stage.STARTUP_SCAN)
        rt.blocked_edges = [Edge(a=1, b=2), Edge(a=4, b=7)]
        self.m._commit_map()
        self.assertEqual(c.stage, Stage.MOVE_TO_QR)

    def test_empty_manual_selection_still_requires_current_round_map(self):
        self.configure_ready(routes=())
        c = self.m.ctx
        c.state, c.stage = 'RUNNING', Stage.STARTUP_SCAN
        self.m._enter_stage()
        self.assertEqual(c.substate, 'WAIT_MAP')
        rt = self.m.runtime['route'][0]
        rt.map_ready, rt.map_revision, rt.run_id = True, 1, 'previous-run'
        self.m._commit_map()
        self.assertEqual(c.stage, Stage.STARTUP_SCAN)
        rt.run_id = c.run_id
        self.m._commit_map()
        self.assertEqual(c.stage, Stage.MOVE_TO_QR)

    def test_map_timeout_faults_even_without_scan_task(self):
        self.scan_stage()
        self.m._map_started_at = time.monotonic() - self.m.init_timeout - 1
        self.m._commit_map()
        self.assertEqual(self.m.ctx.state, 'FAULT')
        self.assertIn('地图提交超时', self.m.ctx.reason)

    def test_work_config_or_stale_feedback_still_blocks_readiness(self):
        self.configure_ready()
        self.m.ctx.state = 'INITIALIZING'
        self.m.runtime['work'][0].config_ready = False
        self.m._initialize()
        self.assertEqual(self.m.ctx.state, 'INITIALIZING')
        rt, _ = self.m.runtime['work']
        self.m.runtime['work'] = (rt, time.monotonic() - 10)
        self.assertFalse(self.m._snapshots_ready())

    def test_pause_and_reset_use_only_enabled_executors_and_clear_manual_map(self):
        self.scan_stage()
        self.m._pause(Trigger.Request(), Trigger.Response())
        self.snapshots()  # Stop confirmation must follow the pause request.
        self.m._settle_stop()
        self.assertEqual(self.m.ctx.state, 'PAUSED')
        old_run = self.m.ctx.run_id
        result = self.m._reset(Trigger.Request(), Trigger.Response())
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.m.ctx.state, 'WAIT_SETUP')
        self.assertNotEqual(self.m.ctx.run_id, old_run)
        self.assertEqual(self.m.ctx.manual_blocked_edges, ())
        self.assertEqual(self.m.ctx.map_revision, 0)

    def test_enabled_mode_keeps_real_lidar_dependency_and_union(self):
        enabled = MissionNode(parameter_overrides=[Parameter('use_lidar', value=True)],
                              cli_args=['--ros-args', '-r', '/C/controll:=/lidar_enabled_test/control'])
        try:
            self.assertIsNotNone(enabled.task_clients.scan)
            self.assertIn('lidar', enabled.task_clients.get)
            enabled.runtime = dict(self.m.runtime)
            self.assertFalse(enabled._snapshots_ready())
            enabled.ctx.state, enabled.ctx.stage = 'RUNNING', Stage.STARTUP_SCAN
            enabled.ctx.manual_blocked_edges = ((4, 7),)
            enabled._send_action = Mock()
            enabled._enter_stage()
            enabled._send_action.assert_called_once()
            task = enabled.ctx.active
            enabled.sessions['lidar'] = 'real-lidar-session'
            result = SimpleNamespace(
                run_id=enabled.ctx.run_id, task_id=task.task_id, source='REAL',
                executor_session_id='real-lidar-session', outcome='COMPLETED',
                travel_ready=True, map_revision=4, blocked_edges=[Edge(a=7, b=4), Edge(a=1, b=2)])
            enabled._commit_map = Mock()
            enabled._action_result(task, SimpleNamespace(status=4, result=result))
            self.assertEqual(enabled.ctx.blocked_edges, ((1, 2), (4, 7)))
            self.assertEqual(enabled.ctx.map_revision, 4)
            enabled._commit_map.assert_called_once()
        finally:
            enabled.destroy_node()


if __name__ == '__main__':
    unittest.main()
