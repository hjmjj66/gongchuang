"""Exercise mission requests against the real planner service without vehicle hardware."""
import math
import time
import unittest
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from agv_interfaces.action import RunWorkBatch
from agv_interfaces.msg import BlockedEdges, CBoardStatus, ExecutorRuntime, OdometryFeedback, FirstLayerRecord, WorkStepRecord
from agv_interfaces.srv import ResetExecutor
from agv_mission.context import ActiveTask, MissionContext
from agv_mission.mission_node import MissionNode
from agv_mission.stages import Stage, expected_steps
from agv_mission.task_code import parse_task_code
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

    def feedback(self, x=0., y=0., yaw=0.):
        self.p._contract.last_lease = time.monotonic()
        board = CBoardStatus(odom_session_id='parking-test', motion_epoch=1,
                            motion_allowed=True, origin_initialized=True)
        board.header.stamp = self.p.get_clock().now().to_msg()
        self.p._on_board(board)
        self.m._on_board(board)
        odom = OdometryFeedback(odom_session_id='parking-test', motion_epoch=1,
                                valid=True, origin_initialized=True, x_m=x, y_m=y,
                                yaw_deg=yaw, yaw_rad=math.radians(yaw))
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

    def test_deferred_work_result_hands_actual_offset_to_next_route(self):
        for stage in (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2, Stage.STORAGE_PLACE_1):
            with self.subTest(stage=stage):
                storage = stage == Stage.STORAGE_PLACE_1
                self.prepare(1, 1, 8 if storage else 6, 'DOWN' if storage else 'RIGHT')
                self.m.ctx.state = 'RUNNING'
                self.m.ctx.stage = stage
                self.m.ctx.code = parse_task_code('156+123+516+231')
                self.m.ctx.map_revision = 1
                self.p._on_map(BlockedEdges(run_id=self.m.ctx.run_id, map_revision=1))
                self.p._work_headings.update({4: 'LEFT', 8: 'DOWN'})
                operation = 'STORAGE_PLACE' if storage else 'COARSE_PROCESS'
                spec = expected_steps(operation, [1,5,6], [1,2,3])
                task = ActiveTask(f'exit-{stage}', 'work', 'work', accepted=True, spec=spec,
                                  batch=2 if stage == Stage.COARSE_BATCH_2 else 1,
                                  defer_reference_return=True)
                self.m.ctx.active = task
                self.m.sessions['work'] = 'work-executor'
                # Work moved the chassis while the planner kept its station target at (0,0).
                self.feedback(-.02, -.03)
                self.snapshot()
                result = RunWorkBatch.Result(
                    run_id=self.m.ctx.run_id, task_id=task.task_id, task_revision=1,
                    executor_session_id='work-executor', source='REAL', outcome='COMPLETED',
                    all_items_completed=True, reference_return_deferred=True,
                    motion_released=True, arm_idle=True, travel_ready=True,
                    final_alignment_outcome='NOT_RUN' if storage else 'ALIGNED',
                    confirmed_steps=[WorkStepRecord(run_id=self.m.ctx.run_id, task_id=task.task_id,
                        step_id=sid, operation=op, item_index=i, color=c, position=p,
                        batch=task.batch, controller_completed=True) for sid,op,i,c,p in spec],
                    first_layer_records=[FirstLayerRecord(color=c, position=p)
                        for c,p in self.m.ctx.code.first_layer.items()] if storage else [])
                self.m._action_result(task, RunWorkBatch.Impl.GetResultService.Response(
                    status=GoalStatus.STATUS_SUCCEEDED, result=result))
                self.assertEqual(int(self.m.ctx.stage), int(stage)+1)
                self.assertEqual(self.p._target_position, (0.,0.))
                self.m._enter_stage()
                deadline = time.monotonic()+3.
                while self.m.pending:
                    self.assertLess(time.monotonic(), deadline)
                    self.executor.spin_once(timeout_sec=.01)
                    self.m._process_pending()
                self.assertEqual(self.m.ctx.state, 'RUNNING', self.m.ctx.reason)
                self.assertTrue(self.m.ctx.active.accepted)
                self.assertGreaterEqual(len(self.p._segments), 3)  # Two translations then work heading.
                first_target = None
                for index, segment in enumerate(self.p._segments):
                    self.feedback(*self.p._position, self.p._yaw_deg)
                    self.p._tick()
                    if index == 0:
                        first_target = self.p._target_position
                        self.assertIn(0., first_target)  # Target was not rebased on the work offset.
                        self.assertNotEqual(first_target, self.p._position)
                    self.feedback(*self.p._target_position, self.p._target_yaw_deg)
                    self.p._tick()
                self.assertFalse(self.p._active)
                self.snapshot()
                self.m._verify_route()
                self.assertEqual(self.m.ctx.stage, Stage.RAW_BATCH_2 if storage else
                                 Stage.STORAGE_PLACE_1 if stage == Stage.COARSE_BATCH_1 else Stage.STORAGE_STACK_2)

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
