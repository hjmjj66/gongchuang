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
from agv_planning.motion import AnglePID, PositionPID
from agv_planning.planner import Opcode, Step, Topology
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


class AnglePIDTests(unittest.TestCase):
    def test_signed_output_limit_and_reverse_correction(self):
        pid = AnglePID(2., 0., .1, 120., .2)
        self.assertEqual(pid.update(90., 0., .02), 120.)
        self.assertEqual(pid.update(-90., 0., .02), -120.)
        self.assertEqual(pid.update(-1., 0., .02), -2.)

    def test_integral_uses_elapsed_time_and_resets_in_tolerance(self):
        pid = AnglePID(1., 2., 0., 120., .2)
        self.assertEqual(pid.update(10., 0., .1), 12.)
        self.assertEqual(pid.update(10., 0., .3), 18.)
        self.assertEqual(pid.update(.2, 5., .02), 0.)
        self.assertEqual(pid.integral, 0.)
        self.assertEqual(pid.update(-10., 0., .1), -12.)

    def test_saturation_does_not_accumulate_integral(self):
        pid = AnglePID(2., 1., 0., 120., .2)
        for _ in range(100):
            self.assertEqual(pid.update(90., 0., .02), 120.)
        self.assertEqual(pid.integral, 0.)
        self.assertLess(pid.update(-1., 0., .02), 0.)

    def test_derivative_damps_measured_rate_without_target_kick(self):
        pid = AnglePID(2., 0., .1, 120., .2)
        self.assertEqual(pid.update(10., 30., .02), 17.)
        self.assertEqual(pid.update(-10., -30., .02), -17.)
        self.assertEqual(pid.update(20., 0., .02), 40.)


class PositionPIDTests(unittest.TestCase):
    def test_independent_limits_in_every_quadrant(self):
        for errors in ((1., 2.), (1., -2.), (-1., 2.), (-1., -2.), (0., 2.)):
            with self.subTest(errors=errors):
                pid = PositionPID(2., 0., .1, (.7, .4), .003)
                vx, vy = pid.update(errors, (0., 0.), .02)
                self.assertAlmostEqual(vx, math.copysign(.7, errors[0]) if errors[0] else 0.)
                self.assertAlmostEqual(vy, math.copysign(.4, errors[1]))
                self.assertGreater(vx * errors[0] + vy * errors[1], 0.)

    def test_separate_integrals_elapsed_time_and_axis_deadband(self):
        pid = PositionPID(1., 2., 0., (.7, .4), .003)
        vx, vy = pid.update((.1, -.05), (0., 0.), .1)
        self.assertAlmostEqual(vx, .12)
        self.assertAlmostEqual(vy, -.06)
        vx, vy = pid.update((.1, -.05), (0., 0.), .3)
        self.assertAlmostEqual(vx, .18)
        self.assertAlmostEqual(vy, -.09)
        vx, vy = pid.update((.002, -.05), (.2, 0.), .1)
        self.assertEqual(vx, 0.)
        self.assertLess(vy, 0.)
        self.assertEqual(pid.integral[0], 0.)
        pid.reset()
        self.assertEqual(pid.integral, [0., 0.])

    def test_axis_saturation_does_not_accumulate_integral(self):
        pid = PositionPID(1., 1., 0., (.7, .4), .003)
        for _ in range(100):
            command = pid.update((1., 1.), (0., 0.), .02)
            self.assertEqual(command, (.7, .4))
        self.assertEqual(pid.integral, [0., 0.])
        vx, vy = pid.update((-.01, .01), (0., 0.), .02)
        self.assertLess(vx, 0.)
        self.assertGreater(vy, 0.)

    def test_saturated_axis_does_not_limit_other_axis_or_its_integral(self):
        pid = PositionPID(1., 1., 0., (.7, .4), .003)
        vx, vy = pid.update((1., .1), (0., 0.), .1)
        self.assertEqual(vx, .7)
        self.assertAlmostEqual(vy, .11)
        self.assertEqual(pid.integral[0], 0.)
        self.assertAlmostEqual(pid.integral[1], .01)
        vx, vy = pid.update((.1, -1.), (0., 0.), .1)
        self.assertAlmostEqual(vx, .11)
        self.assertEqual(vy, -.4)

    def test_derivative_uses_measured_velocity(self):
        pid = PositionPID(2., 0., .1, (.7, .4), .003)
        vx, vy = pid.update((.1, -.1), (.3, -.3), .02)
        self.assertAlmostEqual(vx, .17)
        self.assertAlmostEqual(vy, -.17)

    def test_default_gains_converge_with_velocity_lag(self):
        pid = PositionPID(2., 0., .1, (.7, .4), .003)
        target = (1.2, -.2)
        position, velocity = [0., .1], [0., 0.]
        for _ in range(600):
            errors = tuple(goal - actual for goal, actual in zip(target, position))
            command = pid.update(errors, velocity, .02)
            self.assertLessEqual(abs(command[0]), .7)
            self.assertLessEqual(abs(command[1]), .4)
            if all(abs(error) <= .003 for error in errors):
                break
            for axis in range(2):
                velocity[axis] += (command[axis] - velocity[axis]) * .2
                position[axis] += velocity[axis] * .02
        else:
            self.fail(f'position PID did not converge: {position}')


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
        motion_params = {key: value for key, value in config['route_planner_node']['ros__parameters'].items()
                         if key.startswith(('angle_', 'translation_')) or key == 'turn_arrival_tolerance_deg'}
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('require_map_snapshot',value=False),
            Parameter('debug_allow_any_pose',value=True),
            *[Parameter(key, value=value) for key, value in motion_params.items()],
            *[Parameter(f'work_heading.{key}', value=value)
              for key, value in work_headings.items()]])
        self.feedback()

    def tearDown(self):
        self.p.destroy_node()

    def feedback(self,x=0.,y=0.,yaw=0.,session='one',epoch=1,allowed=True,wz_deg_s=0.,vx=0.,vy=0.):
        st=CBoardStatus(odom_session_id=session,motion_epoch=epoch,
                       motion_allowed=allowed,origin_initialized=True)
        st.header.stamp=self.p.get_clock().now().to_msg()
        self.p._on_board(st)
        od=OdometryFeedback(odom_session_id=session,motion_epoch=epoch,valid=True,
                            origin_initialized=True,x_m=x,y_m=y,yaw_rad=math.radians(yaw),yaw_deg=yaw,
                            wz_rad_s=math.radians(wz_deg_s),vx_mps=vx,vy_mps=vy)
        od.header.stamp=self.p.get_clock().now().to_msg()
        self.p._on_odometry(od)

    def start_route(self):
        req=PlanRoute.Request(request_id='task',start=0,goal=2)
        result=self.p._on_plan(req,PlanRoute.Response()).plan
        self.assertEqual(result.status,'ok',result.reason)
        self.p._tick()

    def test_sideways_does_not_complete_forward_route(self):
        self.p._publish_control = Mock()
        self.start_route()
        self.feedback(x=0.,y=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=-.1,y=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=1.2,y=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        vx, vy, _ = self.p._publish_control.call_args.args
        self.assertEqual(vx, 0.)
        self.assertLess(vy, 0.)
        self.feedback(x=1.2,y=.002)
        self.p._tick()
        self.assertFalse(self.p._active)
        self.assertEqual(self.p._last_state,'COMPLETED')

    def test_node_one_to_two_sends_only_forward_vx(self):
        self.p._publish_control = Mock()
        self.start_route()
        vx, vy, omega = self.p._publish_control.call_args.args
        self.assertGreater(vx, 0.)
        self.assertEqual((vy, omega), (0., 0.))

    def test_every_translation_reverses_both_axis_errors_after_overshoot(self):
        self.p._publish_control = Mock()
        for opcode in TRANSLATIONS:
            with self.subTest(opcode=opcode):
                self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
                self.feedback()
                self.p._start_execution('overshoot', [Step(opcode, 1, 2, 1200, 'DOWN')], 2)
                self.p._tick()
                target_x, target_y = self.p._target_position
                self.feedback(x=target_x + .01, y=target_y - .02)
                self.p._tick()
                self.assertTrue(self.p._active)
                vx, vy, omega = self.p._publish_control.call_args.args
                self.assertAlmostEqual(vx, -.02)
                self.assertAlmostEqual(vy, .04)
                self.assertEqual(omega, 0.)
                self.feedback(x=target_x + .002, y=target_y - .002)
                self.p._tick()
                self.assertEqual(self.p._last_state, 'COMPLETED')

    def test_position_target_survives_route_boundaries_without_accumulating_error(self):
        self.start_route()
        self.feedback(x=1.198, y=.002)
        self.p._tick()
        self.assertFalse(self.p._active)
        req = PlanRoute.Request(request_id='next-position', goal=3)
        self.assertEqual(self.p._on_plan(req, PlanRoute.Response()).plan.status, 'ok')
        self.p._tick()
        self.assertEqual(self.p._target_position, (2.4, 0.))
        self.feedback(x=2.398, y=.002)
        self.p._tick()
        self.assertEqual(self.p._last_state, 'COMPLETED')

    def test_measured_velocity_uses_same_frame_as_position_pid(self):
        self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
        self.feedback(yaw=90.)
        self.p._publish_control = Mock()
        self.start_route()
        body_velocity = body_to_fixed(.3, -.3, math.radians(-10.))
        self.feedback(x=-.1, y=1.1, yaw=100., vx=body_velocity[0], vy=body_velocity[1])
        self.p._tick()
        vx, vy, _ = self.p._publish_control.call_args.args
        goal_velocity = body_to_fixed(vx, vy, math.radians(10.))
        self.assertAlmostEqual(goal_velocity[0], .17)
        self.assertAlmostEqual(goal_velocity[1], -.17)

    def test_configured_translation_limit_and_position_tolerance(self):
        self.p.destroy_node()
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('translation_max_vx_mps', value=.1),
            Parameter('translation_max_vy_mps', value=.04),
            Parameter('translation_arrival_tolerance_mm', value=.5),
            Parameter('require_map_snapshot', value=False)])
        self.p._publish_control = Mock()
        self.feedback()
        self.start_route()
        self.assertAlmostEqual(math.hypot(*self.p._publish_control.call_args.args[:2]), .1)
        self.feedback(x=1.199)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=1.1996, y=.0004)
        self.p._tick()
        self.assertEqual(self.p._last_state, 'COMPLETED')

    def test_configured_limits_apply_to_both_body_axes_after_rotation(self):
        self.p.destroy_node()
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('translation_max_vx_mps', value=.12),
            Parameter('translation_max_vy_mps', value=.035)])
        self.p._control_publisher = Mock()
        for opcode in TRANSLATIONS:
            for yaw in (0., -45., 45., 90.):
                with self.subTest(opcode=opcode, yaw=yaw):
                    self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
                    self.feedback()
                    self.p._start_execution('body-limit', [Step(opcode, 1, 2, 1200, 'DOWN')], 2)
                    self.p._tick()
                    self.feedback(yaw=yaw)
                    self.p._tick()
                    command = self.p._control_publisher.publish.call_args.args[0].command
                    self.assertLessEqual(abs(command.vx_mps), .12)
                    self.assertLessEqual(abs(command.vy_mps), .035)
                    if yaw == 0.:
                        dx, dy = TRANSLATIONS[opcode]
                        self.assertAlmostEqual(command.vx_mps, math.copysign(.12, dx) if dx else 0.)
                        self.assertAlmostEqual(command.vy_mps, math.copysign(.035, dy) if dy else 0.)

    def test_all_translations_hold_heading_and_enforce_body_limits(self):
        self.p._publish_control = Mock()
        for opcode, direction in TRANSLATIONS.items():
            for yaw in (-10., 10.):
                with self.subTest(opcode=opcode, yaw=yaw):
                    self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
                    self.feedback()
                    self.p._start_execution('head-hold', [Step(opcode, 1, 2, 1200, 'DOWN')], 2)
                    self.p._tick()
                    self.feedback(yaw=yaw)
                    self.p._tick()
                    vx, vy, omega = self.p._publish_control.call_args.args
                    self.assertLess(omega * yaw, 0.)
                    self.assertLessEqual(abs(omega), 120.)
                    self.assertLessEqual(abs(vx), self.p._vx_max)
                    self.assertLessEqual(abs(vy), self.p._vy_max)
                    fixed_velocity = body_to_fixed(vx, vy, math.radians(yaw))
                    self.assertGreater(sum(value * axis for value, axis in
                                           zip(fixed_velocity, direction)), 0.)
                    self.assertEqual(self.p._target_yaw_deg, 0.)

    def test_translation_waits_for_angle_after_distance_arrives(self):
        self.p._publish_control = Mock()
        self.start_route()
        self.feedback(x=1.2, yaw=2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        vx, vy, omega = self.p._publish_control.call_args.args
        self.assertEqual((vx, vy), (0., 0.))
        self.assertLess(omega, 0.)
        self.p._position_progress_time = [time.monotonic() - 3.] * 2
        self.feedback(x=1.2, yaw=self.p._turn_tolerance_deg * 2.)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(x=1.2, yaw=.1)
        self.p._tick()
        self.assertEqual(self.p._last_state, 'COMPLETED')
        self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., 0.))

    def test_turn_overshoot_reverses_and_uses_continuous_yaw(self):
        self.p._publish_control = Mock()
        for opcode, sign in ((Opcode.TURN_LEFT_90, 1.), (Opcode.TURN_RIGHT_90, -1.)):
            with self.subTest(opcode=opcode):
                self.p._reset_context(ResetExecutor.Request(home_node=1, initial_heading='DOWN'))
                self.feedback(yaw=350.)
                self.p._start_execution('turn', [Step(opcode, 1, 1, 0, 'UP')] * 2, 1)
                self.p._tick()
                target = 350. + sign * 180.
                self.assertEqual(self.p._target_yaw_deg, target)
                self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., sign * 120.))
                self.feedback(yaw=target + sign * 5.)
                self.p._tick()
                self.assertTrue(self.p._active)
                self.assertLess(self.p._publish_control.call_args.args[2] * sign, 0.)
                self.feedback(yaw=target + sign * .1)
                self.p._tick()
                self.assertEqual(self.p._last_state, 'COMPLETED')

    def test_target_survives_segment_and_route_boundaries(self):
        self.p._publish_control = Mock()
        steps = [Step(Opcode.TURN_LEFT_90, 1, 1, 0, 'RIGHT'),
                 Step(Opcode.FORWARD_ONE_EDGE, 1, 2, 1200, 'RIGHT')]
        self.p._start_execution('turn-then-forward', steps, 2)
        self.p._tick()
        self.feedback(yaw=90.1)
        self.p._tick()
        self.p._tick()
        self.assertEqual(self.p._target_yaw_deg, 90.)
        self.assertAlmostEqual(self.p._direction[0], 0.)
        self.assertAlmostEqual(self.p._direction[1], 1.)
        self.feedback(y=1.2, yaw=90.1)
        self.p._tick()
        req = PlanRoute.Request(request_id='next-route', goal=3)
        self.assertEqual(self.p._on_plan(req, PlanRoute.Response()).plan.status, 'ok')
        self.p._tick()
        self.assertEqual(self.p._target_yaw_deg, 90.)

    def test_angle_stall_aborts_even_while_translation_advances(self):
        self.p._publish_control = Mock()
        self.start_route()
        self.feedback(x=.1, yaw=5.)
        self.p._tick()
        self.p._angle_progress_time = time.monotonic() - 3.
        self.feedback(x=.2, yaw=5.)
        self.p._tick()
        self.assertEqual(self.p._last_state, 'ABORTED')
        self.assertIn('yaw error', self.p._last_reason)
        self.assertEqual(self.p._publish_control.call_args.args[:3], (0., 0., 0.))

    def test_angular_rate_feedback_is_converted_to_degrees(self):
        self.p._publish_control = Mock()
        self.start_route()
        self.feedback(yaw=-10., wz_deg_s=30.)
        self.p._tick()
        self.assertAlmostEqual(self.p._publish_control.call_args.args[2], 17.)

    def test_configured_speed_limit_and_small_angle_tolerance(self):
        self.p.destroy_node()
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('angle_max_speed_deg_s', value=12.),
            Parameter('turn_arrival_tolerance_deg', value=.01)])
        self.p._publish_control = Mock()
        self.feedback()
        self.p._start_execution('precise-turn', [Step(Opcode.TURN_LEFT_90, 1, 1, 0, 'RIGHT')], 1)
        self.p._tick()
        self.assertEqual(self.p._publish_control.call_args.args[2], 12.)
        self.feedback(yaw=89.95)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.p._angle_progress_time = time.monotonic() - 3.
        self.feedback(yaw=89.97)
        self.p._tick()
        self.assertTrue(self.p._active)
        self.feedback(yaw=90.005)
        self.p._tick()
        self.assertEqual(self.p._last_state, 'COMPLETED')

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
                        self.assertLessEqual(abs(vx), self.p._vx_max)
                        self.assertLessEqual(abs(vy), self.p._vy_max)
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
        self.assertLess(vx, 0.)
        self.assertLess(vy, 0.)
        self.assertEqual(omega, 0.)
        self.feedback(x=.2, y=0.)
        self.p._tick()
        vx, vy, omega = self.p._publish_control.call_args.args
        self.assertEqual((vx, omega), (0., 0.))
        self.assertLess(vy, 0.)
        self.feedback(x=.2, y=-.2)
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
        self.assertLess(abs(vx), self.p._vx_max)
        self.assertLess(abs(vy), self.p._vy_max)
        self.p._position_progress_time = [time.monotonic() - 3.] * 2
        self.feedback(x=.19, y=-.15)
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
        self.p._position_progress_time=[time.monotonic()-3.] * 2
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
