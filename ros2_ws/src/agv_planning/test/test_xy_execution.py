"""ROS node contract checks without DDS/vehicle motion: invoke actual callbacks."""
import math
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from agv_interfaces.msg import CBoardStatus, OdometryFeedback
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
        self.p = RoutePlannerNode(parameter_overrides=[
            Parameter('require_map_snapshot',value=False),
            Parameter('debug_allow_any_pose',value=True)])
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
