"""Verify REAL chassis invalidation and MOCK isolation without moving hardware."""
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from agv_interfaces.msg import CBoardStatus, ControlCommand, RouteControl
from agv_mission.context import ActiveTask
from agv_mission.mission_node import MissionNode
from agv_interfaces.action import RunWorkBatch
from agv_mission.stages import Stage
from agv_mission.task_code import parse_task_code


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

    def test_serial_loss_preserves_mission_and_blocks_motion_until_reconnected(self):
        active = self.m.ctx.active
        msg = CBoardStatus(odom_session_id='session-one', motion_epoch=1,
                           origin_initialized=True, fault_reason='serial disconnected')
        msg.header.stamp = self.m.get_clock().now().to_msg()
        self.m._on_board(msg)
        self.m._on_route_control(self.command())
        self.m._maintain_control()
        self.assertEqual(self.m.ctx.state, 'RUNNING')
        self.assertIs(self.m.ctx.active, active)
        self.assertEqual(self.m.control_pub.publish.call_args.args[0].vx_mps, 0.)
        self.board()
        command = self.command()
        command.sequence = 2
        self.m._on_route_control(command)
        self.m.control_pub.publish.assert_called_with(command.command)
        self.assertEqual(self.m.ctx.state, 'RUNNING')

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

    def test_coarse_velocity_is_forwarded_only_during_coarse_and_watchdog_remains_active(self):
        self.m.ctx.stage=Stage.COARSE_BATCH_1
        self.m.ctx.active=ActiveTask('coarse-one','work','work',accepted=True)
        self.m.sessions['work']='vision-one'
        msg=self.command()
        msg.request_id='coarse-one'
        msg.executor_session_id='vision-one'
        self.m._on_coarse_control(msg)
        self.m.control_pub.publish.assert_called_with(msg.command)
        self.m._maintain_control()
        self.assertIs(self.m._route_control,msg)
        self.m._route_control_received=time.monotonic()-1.
        self.m._maintain_control()
        self.assertEqual(self.m.ctx.state,'FAULT')

    def test_work_goals_do_not_request_legacy_fixed_offset(self):
        self.m.ctx.code = parse_task_code('156+213+516+231')
        self.m.ctx.first_layer = self.m.ctx.code.first_layer
        self.m._send_action = Mock()
        for stage in (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2, Stage.RAW_BATCH_1, Stage.STORAGE_PLACE_1,
                      Stage.STORAGE_STACK_2):
            self.m.ctx.stage = stage
            self.m._enter_stage()
            goal = self.m._send_action.call_args.args[1]
            self.assertEqual(goal.camera_car_offset, 0.)
            self.assertEqual(goal.defer_reference_return,
                             stage in (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2, Stage.STORAGE_PLACE_1, Stage.RAW_BATCH_1, Stage.RAW_BATCH_2))
            self.assertEqual(self.m.ctx.active.defer_reference_return, goal.defer_reference_return)

    def test_merge_exit_can_be_disabled_and_mock_navigation_never_requests_it(self):
        self.m.ctx.code = parse_task_code('156+213+516+231')
        self.m._send_action = Mock()
        self.m.ctx.stage = Stage.COARSE_BATCH_1
        self.m.merge_exit_navigation = False
        self.m._enter_stage()
        self.assertFalse(self.m._send_action.call_args.args[1].defer_reference_return)
        self.m.merge_exit_navigation = True
        self.m.expected_source['route'] = 'MOCK'
        self.m._enter_stage()
        self.assertFalse(self.m._send_action.call_args.args[1].defer_reference_return)

    def test_deferred_result_requires_matching_request_and_original_alignment_checks(self):
        result = RunWorkBatch.Result(reference_return_deferred=True, final_alignment_outcome='ALIGNED')
        for stage in (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2, Stage.STORAGE_PLACE_1, Stage.RAW_BATCH_1, Stage.RAW_BATCH_2):
            with self.subTest(stage=stage):
                self.m.ctx.stage = stage
                result.final_alignment_outcome = 'NOT_RUN' if stage in (Stage.STORAGE_PLACE_1, Stage.RAW_BATCH_1, Stage.RAW_BATCH_2) else 'ALIGNED'
                self.m.ctx.active.defer_reference_return = False
                self.assertFalse(self.m._reference_accepted(result))
                self.m.ctx.active.defer_reference_return = True
                self.assertTrue(self.m._reference_accepted(result))
                result.reference_restored = True
                self.assertFalse(self.m._reference_accepted(result))
                result.reference_restored = False
                result.final_alignment_outcome = 'TIMEOUT_SKIPPED'
                self.assertEqual(self.m._reference_accepted(result), stage in (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2))
        self.m.ctx.active.defer_reference_return = False
        for stage in (Stage.RAW_BATCH_1, Stage.STORAGE_STACK_2):
            self.m.ctx.stage = stage
            result.final_alignment_outcome = 'NOT_RUN'
            self.assertFalse(self.m._reference_accepted(result))

    def test_coarse_timeout_exception_does_not_relax_other_area_results(self):
        result=RunWorkBatch.Result(reference_restored=False,final_alignment_outcome='TIMEOUT_SKIPPED')
        self.m.ctx.stage=Stage.COARSE_BATCH_1
        self.assertFalse(self.m._reference_accepted(result))
        result.reference_restored=True
        self.assertTrue(self.m._reference_accepted(result))
        result.final_alignment_outcome='NOT_RUN'
        self.assertFalse(self.m._reference_accepted(result))
        result.final_alignment_outcome='ALIGNED'
        result.reference_restored=False
        self.assertFalse(self.m._reference_accepted(result))
        result.reference_restored=True
        self.assertTrue(self.m._reference_accepted(result))
        self.m.ctx.stage=Stage.STORAGE_PLACE_1
        result.reference_restored=False
        result.final_alignment_outcome='TIMEOUT_SKIPPED'
        self.assertFalse(self.m._reference_accepted(result))

    def test_raw_speed_accepts_only_raw_topic_and_keeps_watchdog(self):
        for stage in (Stage.RAW_BATCH_1, Stage.RAW_BATCH_2):
            with self.subTest(stage=stage):
                self.m.ctx.state='RUNNING'
                self.m.ctx.stage=stage
                self.m.ctx.active=ActiveTask('raw-one','work','work',accepted=True)
                self.m.sessions['work']='vision-one'
                self.m._route_control_sequence=0
                self.m._stop_control()
                msg=self.command()
                msg.request_id='raw-one'
                msg.executor_session_id='vision-one'
                self.m._on_coarse_control(msg)
                self.m._on_storage_control(msg)
                self.assertIsNone(self.m._route_control)
                self.m._on_raw_control(msg)
                self.m.control_pub.publish.assert_called_with(msg.command)
                self.assertIs(self.m._route_control,msg)
                self.m._maintain_control()
                self.assertEqual(self.m.ctx.state,'RUNNING')
                self.m._route_control_received=time.monotonic()-1.
                self.m._maintain_control()
                self.assertEqual(self.m.ctx.state,'FAULT')

    def test_p0_pid_timeout_allows_next_stage_without_claiming_restoration(self):
        result = RunWorkBatch.Result(reference_return_timed_out=True)
        for stage in (Stage.COARSE_BATCH_1,
                      Stage.COARSE_BATCH_2, Stage.STORAGE_PLACE_1):
            with self.subTest(stage=stage):
                self.m.ctx.stage = stage
                self.m.ctx.active.defer_reference_return = False
                result.final_alignment_outcome = ('TIMEOUT_SKIPPED' if stage in
                    (Stage.COARSE_BATCH_1, Stage.COARSE_BATCH_2) else 'NOT_RUN')
                self.assertTrue(self.m._reference_accepted(result))
                result.reference_restored = True
                self.assertFalse(self.m._reference_accepted(result))
                result.reference_restored = False
        self.m.ctx.stage = Stage.STORAGE_STACK_2
        self.assertFalse(self.m._reference_accepted(result))

    def test_raw_velocity_ignored_during_other_area_and_requires_reference_return(self):
        self.m.ctx.stage=Stage.COARSE_BATCH_1
        self.m.ctx.active=ActiveTask('raw-one','work','work',accepted=True)
        self.m.sessions['work']='vision-one'
        msg=self.command()
        msg.request_id='raw-one'
        msg.executor_session_id='vision-one'
        self.m._on_raw_control(msg)
        self.assertIsNone(self.m._route_control)
        self.m.ctx.stage=Stage.RAW_BATCH_1
        result=RunWorkBatch.Result(reference_restored=False,final_alignment_outcome='TIMEOUT_SKIPPED')
        self.assertFalse(self.m._reference_accepted(result))

    def test_storage_velocity_rejects_other_area_topic_and_old_task(self):
        self.m.ctx.stage=Stage.STORAGE_STACK_2
        self.m.ctx.active=ActiveTask('stack-one','work','work',accepted=True)
        self.m.sessions['work']='vision-one'
        msg=self.command()
        msg.request_id='stack-one'
        msg.executor_session_id='vision-one'
        self.m._on_coarse_control(msg)
        self.assertIsNone(self.m._route_control)
        msg.request_id='old-storage-task'
        self.m._on_storage_control(msg)
        self.assertIsNone(self.m._route_control)
        msg.request_id='stack-one'
        self.m._on_storage_control(msg)
        self.m.control_pub.publish.assert_called_with(msg.command)
        self.m._route_control_received=time.monotonic()-1.
        self.m._maintain_control()
        self.assertEqual(self.m.ctx.state,'FAULT')

    def test_storage_first_visit_requires_return_and_second_visit_does_not_return(self):
        for stage in (Stage.STORAGE_PLACE_1,Stage.STORAGE_STACK_2):
            self.m.ctx.stage=stage
            returns = stage == Stage.STORAGE_PLACE_1
            self.assertTrue(self.m._reference_accepted(RunWorkBatch.Result(
                reference_restored=returns,final_alignment_outcome='NOT_RUN')))
            self.assertFalse(self.m._reference_accepted(RunWorkBatch.Result(
                reference_restored=not returns,final_alignment_outcome='NOT_RUN')))

    def test_raw_both_batches_require_return_or_navigation_handoff(self):
        for stage in (Stage.RAW_BATCH_1,Stage.RAW_BATCH_2):
            self.m.ctx.stage=stage
            result=RunWorkBatch.Result(final_alignment_outcome='NOT_RUN')
            self.assertFalse(self.m._reference_accepted(result))
            result.reference_restored=True
            self.assertTrue(self.m._reference_accepted(result))
            self.m.ctx.active.defer_reference_return=True
            self.assertFalse(self.m._reference_accepted(result))
            result.reference_restored=False
            result.reference_return_deferred=True
            self.assertTrue(self.m._reference_accepted(result))
            result.reference_return_timed_out=True
            self.assertFalse(self.m._reference_accepted(result))
            self.m.ctx.active.defer_reference_return=False


if __name__=='__main__':
    unittest.main()
