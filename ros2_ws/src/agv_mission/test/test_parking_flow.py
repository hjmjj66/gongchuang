"""Exercise mission requests against the real planner service without vehicle hardware."""
import math
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from agv_interfaces.msg import BlockedEdges, CBoardStatus, ExecutorRuntime, OdometryFeedback
from agv_interfaces.srv import ResetExecutor
from agv_mission.context import MissionContext
from agv_mission.mission_node import MissionNode
from agv_mission.stages import Stage
from agv_mission.state_machine import start
from agv_planning.motion import TRANSLATIONS
from agv_planning.route_planner_node import RoutePlannerNode


class ParkingFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.m = MissionNode(parameter_overrides=[Parameter('parking_offset_mm', value=275)],
                             cli_args=['--ros-args', '-r', '/C/controll:=/parking_test/control'])
        self.p = RoutePlannerNode()
        for node in (self.m, self.p):
            for timer in node.timers:
                timer.cancel()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.m)
        self.executor.add_node(self.p)
        self.assertTrue(self.m.task_clients.route.wait_for_service(timeout_sec=3.))

    def tearDown(self):
        self.executor.shutdown()
        self.p.destroy_node()
        self.m.destroy_node()

    def feedback(self, x=0., y=0.):
        self.p._contract.last_lease = time.monotonic()
        board = CBoardStatus(odom_session_id='parking-test', motion_epoch=1,
                            motion_allowed=True, origin_initialized=True)
        board.header.stamp = self.p.get_clock().now().to_msg()
        self.p._on_board(board)
        self.m._on_board(board)
        odom = OdometryFeedback(odom_session_id='parking-test', motion_epoch=1,
                                valid=True, origin_initialized=True, x_m=x, y_m=y)
        odom.header.stamp = self.p.get_clock().now().to_msg()
        self.p._on_odometry(odom)

    def prepare(self, home, finish, node, heading):
        self.m.home, self.m.finish_node = home, finish
        self.m.ctx = MissionContext(home_node=home, state='READY')
        self.p._reset_context(ResetExecutor.Request(home_node=node, initial_heading=heading))
        self.p._contract.run_id = self.m.ctx.run_id
        self.feedback()
        for key in ('route', 'lidar', 'work'):
            self.m.runtime[key] = (ExecutorRuntime(), time.monotonic())
        self.m.sessions['route'] = self.p._contract.session_id
        self.snapshot()

    def snapshot(self):
        self.m.runtime['route'] = (self.p._contract.snapshot(), time.monotonic())

    def run_route_stage(self):
        self.m._enter_stage()
        deadline = time.monotonic() + 3.
        while self.m.pending:
            self.assertLess(time.monotonic(), deadline)
            self.executor.spin_once(timeout_sec=.01)
            self.m._process_pending()
        self.assertEqual(self.m.ctx.state, 'RUNNING', self.m.ctx.reason)
        self.assertTrue(self.m.ctx.active.accepted)
        segments = self.p._segments
        self.assertTrue(segments)
        for segment in segments:
            self.feedback(*self.p._position)
            self.p._tick()
            ux, uy = TRANSLATIONS[segment.opcode]
            x, y = self.p._position
            self.feedback(x + ux * segment.amount / 1000., y + uy * segment.amount / 1000.)
            self.p._tick()
        self.assertFalse(self.p._active)
        self.snapshot()
        self.m._verify_route()
        self.assertTrue(self.m._enter_pending or self.m.ctx.state == 'FINISHED')
        return segments

    def test_exit_precedes_scan_and_keeps_early_qr_for_both_start_zones(self):
        for home, heading, left_sign in ((1, 'DOWN', -1), (3, 'UP', 1)):
            with self.subTest(home=home):
                self.prepare(home, home, home, heading)
                start(self.m.ctx)
                self.assertEqual(self.m.ctx.stage, Stage.EXIT_START_AREA)
                self.m._on_qr(String(data='156+123+516+231'))
                self.assertIsNotNone(self.m.ctx.pending_code)
                segments = self.run_route_stage()
                self.assertEqual(len(segments), 1)
                vx, vy = TRANSLATIONS[segments[0].opcode]
                self.assertGreater(vx, 0.)
                self.assertGreater(vy * left_sign, 0.)
                self.assertAlmostEqual(segments[0].amount, 275 * math.sqrt(2))
                self.assertEqual(self.m.ctx.stage, Stage.STARTUP_SCAN)
                self.assertEqual(self.m.ctx.state, 'RUNNING')
                self.assertFalse(self.p._map_ready)

    def test_return_reaches_corner_then_parks_before_finished_for_all_zone_pairs(self):
        for home in (1, 3):
            for finish in (1, 3):
                with self.subTest(home=home, finish=finish):
                    self.prepare(home, finish, 8, 'RIGHT')
                    self.m.ctx.state = 'RUNNING'
                    self.m.ctx.stage = Stage.RETURN_HOME
                    self.m.ctx.map_revision = 1
                    self.p._on_map(BlockedEdges(run_id=self.m.ctx.run_id, map_revision=1))
                    segments = self.run_route_stage()
                    self.assertTrue(all(not (TRANSLATIONS[s.opcode][0] and TRANSLATIONS[s.opcode][1])
                                        for s in segments))
                    self.assertEqual(self.m.ctx.state, 'RUNNING')
                    self.assertEqual(self.m.ctx.stage, Stage.ENTER_FINISH_AREA)
                    self.assertEqual(self.m.ctx.confirmed_node, finish)
                    self.assertEqual(self.m.ctx.current_heading, 'RIGHT')
                    segments = self.run_route_stage()
                    self.assertEqual(len(segments), 1)
                    vx, vy = TRANSLATIONS[segments[0].opcode]
                    self.assertGreater(vx, 0.)
                    self.assertGreater(vy * (1 if finish == 1 else -1), 0.)
                    self.assertEqual(self.m.ctx.state, 'FINISHED')
                    self.assertTrue(self.m.ctx.quiescent)
                    self.assertEqual(self.m.ctx.current_heading, 'RIGHT')


if __name__ == '__main__':
    unittest.main()
