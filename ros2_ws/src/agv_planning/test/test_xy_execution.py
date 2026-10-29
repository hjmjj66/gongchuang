"""ROS node contract checks without DDS/vehicle motion: invoke actual callbacks."""
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

import rclpy
import yaml
from rclpy.parameter import Parameter
from agv_interfaces.msg import BlockedEdges, CBoardStatus, OdometryFeedback
from agv_interfaces.srv import PlanRoute, ResetExecutor
from agv_planning.odometry import body_to_fixed, projection
from agv_planning.planner import Opcode, Topology
from agv_planning.route_planner_node import RoutePlannerNode, TRANSLATIONS


class GeometryTests(unittest.TestCase):
    def test_all_axes_rotations_signed_progress(self):
        for yaw in (0., math.pi/2, -math.pi/2, math.pi, .61):
            for direction in TRANSLATIONS.values():
                e = body_to_fixed(*direction,yaw)
                start = (4.,-3.)
                current = (start[0]+.4*e[0]-.2*e[1],start[1]+.4*e[1]+.2*e[0])
                along,cross = projection(start,current,e)
                self.assertAlmostEqual(along,400.)
                self.assertAlmostEqual(cross,200.)
                reverse = (start[0]-.3*e[0], start[1]-.3*e[1])
                self.assertAlmostEqual(projection(start,reverse,e)[0],-300.)
        self.assertEqual(TRANSLATIONS[Opcode.FORWARD_ONE_EDGE],(1.,0.))

    def test_forward_left_axes_and_ccw_rotation(self):
        for yaw, forward, left in (
                (0., (1., 0.), (0., 1.)),
                (math.pi/2, (0., 1.), (-1., 0.)),
                (math.pi, (-1., 0.), (0., -1.)),
                (-math.pi/2, (0., -1.), (1., 0.))):
            with self.subTest(yaw=yaw):
                for body, expected in (((1., 0.), forward), ((0., 1.), left)):
                    for actual, value in zip(body_to_fixed(*body, yaw), expected):
                        self.assertAlmostEqual(actual, value)

    def test_map_directions_for_every_initial_heading(self):
        # From center node: UP, RIGHT, DOWN, LEFT on the map.
        for heading, expected in (
                ('UP', ((1,0), (0,-1), (-1,0), (0,1))),
                ('RIGHT', ((0,1), (1,0), (0,-1), (-1,0))),
                ('DOWN', ((-1,0), (0,1), (1,0), (0,-1))),
                ('LEFT', ((0,-1), (-1,0), (0,1), (1,0)))):
            for goal, direction in zip((4, 2, 6, 8), expected):
                with self.subTest(heading=heading, goal=goal):
                    step, = Topology().build_steps((5, goal), heading)
                    self.assertEqual(TRANSLATIONS[step.opcode], direction)


class ExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'config/planner.yaml').read_text())
        work_headings = config['route_planner_node']['ros__parameters']['work_heading']
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('require_map_snapshot',value=False),
            Parameter('debug_allow_any_pose',value=True),
            *[Parameter(f'work_heading.{key}', value=value)
              for key, value in work_headings.items()]])
        self.feedback()

    def tearDown(self):
        self.p.destroy_node()

    def feedback(self,x=0.,y=0.,yaw=0.,session='one',epoch=1,allowed=True):
        st=CBoardStatus(odom_session_id=session,motion_epoch=epoch,
                       motion_allowed=allowed,origin_initialized=True)
        st.header.stamp=self.p.get_clock().now().to_msg()
        self.p._on_board(st)
        od=OdometryFeedback(odom_session_id=session,motion_epoch=epoch,valid=True,
                            origin_initialized=True,x_m=x,y_m=y,yaw_rad=math.radians(yaw),yaw_deg=yaw)
        od.header.stamp=self.p.get_clock().now().to_msg()
        self.p._on_odometry(od)

    def start_route(self):
        req=PlanRoute.Request(request_id='task',start=0,goal=2)
        result=self.p._on_plan(req,PlanRoute.Response()).plan
        self.assertEqual(result.status,'ok',result.reason)
        self.p._tick()

    def test_sideways_does_not_complete_forward_route(self):
        self.start_route()
        self.feedback(x=0.,y=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=-.1,y=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=1.2,y=2.)
        self.p._tick()
        self.assertFalse(self.p._active)
        self.assertEqual(self.p._last_state,'COMPLETED')

    def test_node_one_to_two_sends_only_forward_vx(self):
        self.p._publish_control = Mock()
        self.start_route()
        vx, vy, omega = self.p._publish_control.call_args.args
        self.assertGreater(vx, 0.)
        self.assertEqual((vy, omega), (0., 0.))

    def test_parking_directions_for_both_zones_and_all_headings(self):
        entry_signs = {
            1: {'UP': (1, -1), 'RIGHT': (1, 1), 'DOWN': (-1, 1), 'LEFT': (-1, -1)},
            3: {'UP': (-1, -1), 'RIGHT': (1, -1), 'DOWN': (1, 1), 'LEFT': (-1, 1)},
        }
        self.p._publish_control = Mock()
        for node, headings in entry_signs.items():
            for heading, signs in headings.items():
                for maneuver, multiplier in (('ENTER', 1), ('EXIT', -1)):
                    with self.subTest(node=node, heading=heading, maneuver=maneuver):
                        self.p._reset_context(ResetExecutor.Request(home_node=node, initial_heading=heading))
                        self.feedback(x=3., y=-2., yaw=37.)
                        req = PlanRoute.Request(request_id=f'{node}-{heading}-{maneuver}', goal=node,
                                                parking_maneuver=maneuver, parking_offset_mm=200)
                        plan = self.p._on_plan(req, PlanRoute.Response()).plan
                        self.assertEqual(plan.status, 'ok', plan.reason)
                        self.assertEqual(list(plan.nodes), [node])
                        self.assertEqual((plan.start_heading, plan.end_heading), (heading, heading))
                        self.assertEqual(len(plan.actions), 1)
                        self.assertEqual(plan.actions[0].distance_mm, 200)
                        self.assertEqual(plan.total_distance_mm, 283)
                        self.p._tick()
                        vx, vy, omega = self.p._publish_control.call_args.args
                        self.assertGreater(vx * signs[0] * multiplier, 0.)
                        self.assertGreater(vy * signs[1] * multiplier, 0.)
                        self.assertAlmostEqual(math.hypot(vx, vy), self.p._v_fast)
                        self.assertEqual(omega, 0.)
                        dx, dy = body_to_fixed(.2 * signs[0] * multiplier,
                                               .2 * signs[1] * multiplier, math.radians(37.))
                        self.feedback(x=3. + dx, y=-2. + dy, yaw=37.)
                        self.p._tick()
                        self.assertFalse(self.p._active)
                        self.assertEqual(self.p._last_state, 'COMPLETED')
                        self.assertEqual(self.p._current_heading, heading)
                        self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., 0.))

    def test_diagonal_requires_both_axes_and_stops_arrived_axis(self):
        self.p._publish_control = Mock()
        req = PlanRoute.Request(request_id='both-axes', goal=1,
                                parking_maneuver='EXIT', parking_offset_mm=200)
        self.assertEqual(self.p._on_plan(req, PlanRoute.Response()).plan.status, 'ok')
        self.p._tick()
        # This has the full diagonal projection, but no lateral progress.
        self.feedback(x=.4, y=0.)
        self.p._tick()
        self.assertTrue(self.p._active)
        vx, vy, omega = self.p._publish_control.call_args.args
        self.assertEqual((vx, omega), (0., 0.))
        self.assertLess(vy, 0.)
        self.feedback(x=.4, y=-.2)
        self.p._tick()
        self.assertFalse(self.p._active)

    def test_exit_before_map_then_full_edge_from_corner(self):
        self.p._require_snapshot = True
        req = PlanRoute.Request(request_id='exit-before-map', goal=1,
                                parking_maneuver='EXIT', parking_offset_mm=350)
        plan = self.p._on_plan(req, PlanRoute.Response()).plan
        self.assertEqual(plan.status, 'ok', plan.reason)
        self.p._tick()
        self.feedback(x=.2, y=-.2)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=.35, y=-.35)
        self.p._tick()
        self.assertFalse(self.p._active)
        normal = PlanRoute.Request(request_id='without-map', goal=2)
        self.assertEqual(self.p._on_plan(normal, PlanRoute.Response()).plan.status, 'map_not_ready')
        self.p._on_map(BlockedEdges(map_revision=1))
        self.p._topology = Topology(default_edge_length_mm=900)
        normal.request_id = 'full-edge'
        plan = self.p._on_plan(normal, PlanRoute.Response()).plan
        self.assertEqual(plan.status, 'ok', plan.reason)
        self.assertEqual(plan.total_distance_mm, 900)
        self.assertEqual(plan.actions[0].opcode, Opcode.FORWARD_ONE_EDGE)
        self.p._tick()
        self.feedback(x=.9, y=-.35)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=1.25, y=-.35)
        self.p._tick()
        self.assertFalse(self.p._active)

    def test_diagonal_decelerates_and_detects_stalled_axis(self):
        self.p._publish_control = Mock()
        req = PlanRoute.Request(request_id='diagonal-stall', goal=1,
                                parking_maneuver='EXIT', parking_offset_mm=200)
        self.assertEqual(self.p._on_plan(req, PlanRoute.Response()).plan.status, 'ok')
        self.p._tick()
        self.feedback(x=.15, y=-.15)
        self.p._tick()
        vx, vy, _ = self.p._publish_control.call_args.args
        self.assertGreater(vx, 0.)
        self.assertLess(vy, 0.)
        self.assertLess(math.hypot(vx, vy), self.p._v_fast)
        self.p._segment_progress_time = time.monotonic() - 3.
        self.feedback(x=.4, y=-.15)
        self.p._tick()
        self.assertFalse(self.p._active)
        self.assertEqual(self.p._last_state, 'ABORTED')
        self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., 0.))

    def test_parking_request_validation(self):
        for fields in (
                {'parking_maneuver': 'OTHER', 'goal': 1, 'parking_offset_mm': 200},
                {'parking_maneuver': 'EXIT', 'goal': 3, 'parking_offset_mm': 200},
                {'parking_maneuver': 'EXIT', 'goal': 1, 'parking_offset_mm': 0},
                {'parking_maneuver': 'ENTER', 'goal': 1, 'parking_offset_mm': 1},
                {'parking_maneuver': 'ENTER', 'goal': 1, 'parking_offset_mm': 200, 'align_at_goal': True},
                {'goal': 1, 'parking_offset_mm': 200}):
            with self.subTest(fields=fields):
                plan = self.p._on_plan(PlanRoute.Request(**fields), PlanRoute.Response()).plan
                self.assertEqual(plan.status, 'invalid_request', plan.reason)
                self.assertFalse(self.p._active)

    def test_work_side_faces_station(self):
        # Map axes: x right, y up. QR is on the left of the vehicle;
        # the other stations are on its right.
        yaw = {'UP': math.pi/2, 'RIGHT': 0., 'DOWN': -math.pi/2, 'LEFT': math.pi}
        for node, side, outward in ((2, 1., (1., 0.)), (4, -1., (0., 1.)),
                                    (6, -1., (0., -1.)), (8, -1., (-1., 0.))):
            with self.subTest(node=node):
                facing = body_to_fixed(0., side, yaw[self.p._work_headings[node]])
                for actual, expected in zip(facing, outward):
                    self.assertAlmostEqual(actual, expected)

    def test_in_place_work_alignment_from_every_heading(self):
        self.p._publish_control = Mock()
        for node, target in ((2, 'DOWN'), (4, 'LEFT'), (6, 'RIGHT'), (8, 'DOWN')):
            for initial in ('UP', 'RIGHT', 'DOWN', 'LEFT'):
                with self.subTest(node=node, initial=initial):
                    self.p._reset_context(ResetExecutor.Request(home_node=node,
                                                               initial_heading=initial))
                    self.feedback()
                    req = PlanRoute.Request(request_id=f'align-{node}-{initial}',
                                            goal=node, align_at_goal=True)
                    plan = self.p._on_plan(req, PlanRoute.Response()).plan
                    self.assertEqual(plan.status, 'ok', plan.reason)
                    self.assertEqual(list(plan.nodes), [node])
                    self.assertEqual(plan.end_heading, target)
                    self.assertLessEqual(len(plan.actions), 2)
                    yaw = 0.
                    for segment in self.p._segments:
                        self.assertIn(segment.opcode, (Opcode.TURN_LEFT_90, Opcode.TURN_RIGHT_90))
                        self.p._tick()
                        vx, vy, omega = self.p._publish_control.call_args.args
                        sign = 1. if segment.opcode == Opcode.TURN_LEFT_90 else -1.
                        self.assertEqual((vx, vy), (0., 0.))
                        self.assertGreater(sign * omega, 0.)
                        yaw += sign * segment.amount
                        self.feedback(yaw=yaw)
                        self.p._tick()
                    self.assertFalse(self.p._active)
                    self.assertEqual(self.p._last_state, 'COMPLETED')
                    self.assertEqual(self.p._current_heading, target)
                    self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., 0.))

    def test_work_rotation_only_after_translation(self):
        req = PlanRoute.Request(request_id='aligned-route', goal=4, align_at_goal=True)
        plan = self.p._on_plan(req, PlanRoute.Response()).plan
        self.assertEqual(plan.status, 'ok', plan.reason)
        rotations = [step for step in plan.actions if step.opcode not in TRANSLATIONS]
        self.assertTrue(rotations)
        for step in rotations:
            self.assertEqual((step.from_node, step.to_node), (4, 4))
        self.assertEqual(plan.actions[-1].target_heading, 'LEFT')
        for step in plan.actions[:-len(rotations)]:
            self.assertIn(step.opcode, TRANSLATIONS)
            self.assertEqual(step.target_heading, 'DOWN')

    def test_forward_progress_after_ccw_quarter_turn(self):
        # Establish the map heading at this yaw, as a new explicit UI start does.
        self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
        self.feedback(x=3., y=4., yaw=90.)
        self.start_route()
        self.feedback(x=3., y=5.2, yaw=90.)
        self.p._tick()
        self.assertFalse(self.p._active)
        self.assertEqual(self.p._last_state, 'COMPLETED')

    def test_reset_during_task_aborts_and_requires_context_reset(self):
        self.start_route()
        self.feedback(session='two',epoch=3)
        self.assertFalse(self.p._active)
        self.assertFalse(self.p._position_known)
        self.assertEqual(self.p._last_state,'ABORTED')
        req=PlanRoute.Request(request_id='new-task',start=1,goal=2,current_heading='DOWN')
        result=self.p._on_plan(req,PlanRoute.Response()).plan
        self.assertEqual(result.status,'invalid_request')
        self.p._reset_context(ResetExecutor.Request(home_node=1,initial_heading='DOWN'))
        self.feedback(session='two',epoch=3)
        result=self.p._on_plan(req,PlanRoute.Response()).plan
        self.assertEqual(result.status,'ok',result.reason)

    def test_permission_generation_change_preserves_no_old_task(self):
        self.start_route()
        old=self.p._task_binding
        self.feedback(epoch=2)
        self.assertEqual(self.p._task_binding,old)
        self.assertFalse(self.p._active)
        self.assertTrue(self.p._context_invalid)

    def test_jitter_does_not_keep_progress_watchdog_alive(self):
        self.start_route()
        self.p._segment_progress_time=time.monotonic()-3.
        self.feedback(x=0.,y=1.)
        self.p._tick()
        self.assertFalse(self.p._active)
        self.assertIn('no progress',self.p._last_reason)

    def test_stale_or_invalid_observation(self):
        self.start_route()
        msg=OdometryFeedback(odom_session_id='one',motion_epoch=1,valid=False)
        msg.header.stamp=self.p.get_clock().now().to_msg()
        self.p._on_odometry(msg)
        self.assertFalse(self.p._active)
        self.assertFalse(self.p._feedback_ready())


if __name__=='__main__':
    unittest.main()
