"""Verify REAL chassis invalidation and MOCK isolation without moving hardware."""
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from agv_interfaces.msg import CBoardStatus, ControlCommand, RouteControl
from agv_mission.context import ActiveTask
from agv_mission.mission_node import MissionNode


class ChassisContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.m=MissionNode()
        self.board()
        self.m.ctx.state='RUNNING'
        self.m.ctx.active=ActiveTask('route-one','route','route',accepted=True)
        self.m.sessions['route']='planner-one'
        self.m._snapshots_ready=lambda: True
        self.publisher=self.m.control_pub
        self.m.control_pub=Mock()

    def tearDown(self):
        self.m.control_pub=self.publisher
        self.m.destroy_node()

    def board(self,session='session-one',epoch=1,allowed=True):
        msg=CBoardStatus(odom_session_id=session,motion_epoch=epoch,
                        motion_allowed=allowed,origin_initialized=True)
        msg.header.stamp=self.m.get_clock().now().to_msg()
        self.m._on_board(msg)

    def command(self,session='session-one',epoch=1):
        msg=RouteControl(run_id=self.m.ctx.run_id,request_id='route-one',
                         executor_session_id='planner-one',source='REAL',sequence=1,
                         state='EXECUTING',command=ControlCommand(vx_mps=.1,
                            odom_session_id=session,motion_epoch=epoch))
        msg.header.stamp=self.m.get_clock().now().to_msg()
        return msg

    def test_matching_command_forwarded_without_axis_swap(self):
        msg=self.command()
        self.m._on_route_control(msg)
        self.assertEqual(self.m.ctx.state,'RUNNING')
        self.m.control_pub.publish.assert_called_with(msg.command)
        self.assertEqual(self.m._route_control.command.vx_mps,.1)
        self.assertEqual(self.m._route_control.command.vy_mps,0.)

    def test_old_generation_cannot_be_forwarded_after_recovery(self):
        self.m._on_route_control(self.command(epoch=0))
        self.assertEqual(self.m.ctx.state,'FAULT')
        self.assertIsNone(self.m._route_control)
        self.assertEqual(self.m.control_pub.publish.call_args.args[0].vx_mps,0.)
        self.assertEqual(self.m.control_pub.publish.call_args.args[0].vy_mps,0.)

    def test_reset_while_non_route_stage_stops_competition(self):
        self.m.ctx.active=None
        self.board(session='new-session',epoch=3)
        self.assertEqual(self.m.ctx.state,'FAULT')
        self.assertFalse(self.m.ctx.position_known)
        self.board(session='new-session',epoch=3)
        self.assertEqual(self.m.ctx.state,'FAULT')

    def test_status_silence_invalidates_readiness(self):
        self.m._board_received=time.monotonic()-1.
        self.assertFalse(self.m._chassis_ready())

    def test_mock_route_does_not_depend_on_real_board(self):
        mock=MissionNode(parameter_overrides=[Parameter('mode',value='mission_mock')],
                         cli_args=['--ros-args','-r','__node:=mock_mission_contract'])
        try:
            self.assertIsNone(mock.board_sub)
            self.assertTrue(mock._chassis_ready())
            self.assertNotEqual(mock.control_pub.topic_name,'/C/controll')
        finally:
            mock.destroy_node()


if __name__=='__main__':
    unittest.main()
