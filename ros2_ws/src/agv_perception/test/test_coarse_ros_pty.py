"""Opt-in real ROS/mission/bridge integration using an explicitly simulated PTY peer.

AGV_COARSE_ROS_TEST=1; never opens a real serial port or camera.
"""

import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest

import yaml

from agv_perception.coarse_control import Observation
from test_coarse_control import calibrated_config


class ArmBoardPeer:
    """Test controller: integrates wheel commands and acknowledges timed arm programs."""
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.x = self.y = self.yaw = 0.
        self.command = (0.,0.,0.,0)
        self.arm_until = 0.
        self.jog_at = float('-inf')
        self.jog = False
        self.arm_yaw = self.arm_fb = 0.
        self.jog_values = (0., 0., 0., 0)
        self.reset_work_at = None
        self.frames = []
        self.arm_actions = []
        self.error = None
        self.reject_cmd = None
        self.pos_flags = 7
        self.feedback_enabled = True
        self.clock_start = time.monotonic()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self.run,daemon=True)

    def run(self):
        from agv_cboard.protocol import (
            ACK, ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT,
            ODOM, ODOM_STRUCT, STATUS, STATUS_STRUCT, FrameParser, encode_frame,
        )
        parser = FrameParser()
        start = last = time.monotonic()
        next_send = start
        next_status = start
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                try:
                    data = os.read(self.master, 4096)
                except BlockingIOError:
                    data = b''
                for mid, payload in parser.feed(data, now):
                    self.frames.append((now,mid,payload))
                    if mid == CHASSIS_CMD:
                        self.command = CHASSIS_STRUCT.unpack(payload)
                    elif mid == ARM_CMD:
                        cmd = payload[0]
                        self.arm_actions.append((cmd, self.arm_fb, self.arm_yaw))
                        result=5 if cmd == self.reject_cmd else 0
                        self.arm_until = now+.12 if cmd and not result else now
                        self.jog = False
                        if 2 <= cmd <= 7 and not result:
                            self.reset_work_at = self.arm_until
                        os.write(self.master, encode_frame(ACK,struct.pack('<BBH',ARM_CMD,result,0)))
                    elif mid == ARM_JOG:
                        self.jog_values = JOG_STRUCT.unpack(payload)
                        self.jog = bool(self.jog_values[3])
                        self.jog_at = now
                if now >= next_send:
                    dt = now-last
                    last = now
                    vx,vy,wz = self.command[:3] if self.command[3] else (0.,0.,0.)
                    c,s = math.cos(self.yaw),math.sin(self.yaw)
                    self.x += (c*vx-s*vy)*dt
                    self.y += (s*vx+c*vy)*dt
                    self.yaw += wz*dt
                    if self.jog and now-self.jog_at < .2:
                        self.arm_yaw += self.jog_values[0]*dt
                        self.arm_fb += self.jog_values[1]*dt
                    if self.reset_work_at is not None and now >= self.reset_work_at:
                        self.arm_yaw = self.arm_fb = 0.
                        self.reset_work_at = None
                    tick = int((now-self.clock_start)*1000)+1
                    arm_state = int(now < self.arm_until or (self.jog and now-self.jog_at < .2))
                    packet=encode_frame(ODOM, ODOM_STRUCT.pack(
                        tick,self.x,self.y,self.yaw,vx,vy,wz,13))
                    if now >= next_status:
                        packet=encode_frame(STATUS, STATUS_STRUCT.pack(
                            tick,1,arm_state,255,0,round(self.arm_fb*3200/60),0,
                            self.arm_yaw,2,self.pos_flags))+packet
                        next_status=now+.05
                    if self.feedback_enabled:
                        os.write(self.master,packet)
                    next_send = now+.01
                self._stop.wait(.001)
        except Exception as error:
            self.error = error

    def close(self):
        self._stop.set()
        self.thread.join(2.)
        os.close(self.master)
        os.close(self.slave)


def coarse_scene(board):
    rings = {}
    for ring,x in ((1,.06),(2,0.),(3,-.06)):
        u = 100.+(board.x-x)*200.+board.arm_yaw*100.
        v = 100.-board.y*200.+(u-100.)*math.tan(board.yaw)+board.arm_fb*5.
        rings[ring] = (u,v)
    return rings


def fast_coarse_config():
    p = calibrated_config()
    p['angle_alignment']['pid']['kp'] = 2.
    p['angle_alignment']['max_wz_rad_s'] = .3
    p['base_servo'].update(tolerance_px=1.,max_vx_mps=.2,max_vy_mps=.2)
    for axis in ('u_pid','v_pid'):
        p['base_servo'][axis]['kp'] = .01
    p['gantry_servo'].update(tolerance_px=.5,max_yaw_rad_s=.4,max_fb_rpm=20.,
                             small_yaw_rad_s=.01,small_fb_rpm=.2)
    p['gantry_servo']['u_pid']['kp'] = .02
    p['gantry_servo']['v_pid']['kp'] = .5
    p['gantry_motion'].update(length_tolerance_pulses=3.,angle_tolerance_rad=.005,
                              max_yaw_rad_s=.5,max_fb_rpm=20.)
    p['gantry_motion']['yaw_pid']['kp'] = 4.
    p['gantry_motion']['length_pid']['kp'] = 4.*60/3200
    p['execution'].update(zero_settle_time_s=.04,feedback_timeout_s=.5)
    return p


class VisionFixture:
    """Explicit image observations derived from the simulated board's physical position."""
    ready = True
    error = ''

    def __init__(self, board):
        self.board = board
        self.kind = ''
        self.color = 0
        self.sequence = 0
        self.active = False
        self.owner = None
        self.skip_exit = False
        self.stall_ring_alignment = False

    def request(self, active, kind='', color=0, area='coarse', *, geometry_target=0):
        self.active,self.kind,self.color=active,kind,color
        self.geometry_target = geometry_target

    def latest(self):
        if not self.active or not self.kind:
            return None
        self.sequence+=1
        rings = coarse_scene(self.board)
        position={5:2,1:3,6:1}.get(self.color,2)
        materials=((self.color,rings[position]),) if self.kind=='material' else ()
        if self.skip_exit and self.owner.task and self.owner.task.state == 'ALIGN_EXIT':
            rings={}
        if (self.stall_ring_alignment and self.owner.task and
                self.owner.task.state in ('CENTER_RING2', 'ALIGN_RING')):
            rings={ring:(100.75,100.75) for ring in (1,2,3)}
        return Observation(self.kind,self.sequence,time.monotonic(),(201,201),rings,materials,
                           geometry_target=self.geometry_target)

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_COARSE_ROS_TEST')=='1', 'opt in to isolated ROS + PTY integration')
class CoarseRosTests(unittest.TestCase):
    def test_serial_feedback_loss_resumes_visual_task_and_replays_fixed_arm_via_work_pose(self):
        self.run_scenario('serial_pause')

    def test_complete_two_batches_through_real_mission_and_serial_bridge(self):
        self.run_scenario('two_batches')

    def test_two_batches_stow_and_handoff_without_returning_p0(self):
        self.run_scenario('deferred_exit')

    def test_final_three_second_timeout_continues_and_mission_accepts(self):
        self.run_scenario('exit_timeout')

    def test_fixed_action_rejection_stops_without_retry(self):
        self.run_scenario('reject_place')

    def test_cancel_during_gantry_alignment_sends_stop_and_no_place(self):
        self.run_scenario('cancel_servo')

    def test_invalid_fb_flag_stops_before_recording_or_placing(self):
        self.run_scenario('invalid_position')

    def test_five_second_ring_timeouts_continue_all_placements_and_mission(self):
        self.run_scenario('ring_timeout')

    def test_retrieval_pid_timeout_continues_all_picks_and_mission(self):
        self.run_scenario('retrieval_timeout')

    def test_p0_pid_timeout_reports_actual_result_and_mission_continues(self):
        self.run_scenario('reference_timeout')

    def run_scenario(self, scenario):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from agv_interfaces.action import RunWorkBatch
        from agv_interfaces.srv import ResetExecutor
        from agv_mission.context import ActiveTask
        from agv_mission.mission_node import MissionNode
        from agv_mission.stages import Stage, expected_steps
        from agv_perception.coarse_node import CoarseNode
        from std_msgs.msg import String

        domain = int(os.environ.get('ROS_DOMAIN_ID', '113')) + (
            'two_batches', 'exit_timeout', 'reject_place', 'cancel_servo', 'invalid_position', 'ring_timeout',
            'deferred_exit', 'retrieval_timeout', 'reference_timeout', 'serial_pause').index(scenario)
        rclpy.init(domain_id=domain)
        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)  # Keep the PTY peer responsive to the 20 ms serial write deadline.
        board=ArmBoardPeer()
        board.x, board.y, board.yaw = .012, -.012, .08
        board.pos_flags = 6 if scenario == 'invalid_position' else 1
        if scenario == 'reject_place':
            board.reject_cmd=6
        bridge=coarse=mission=client=executor=action=None
        temp=tempfile.TemporaryDirectory(prefix='coarse_pty_')
        try:
            p=fast_coarse_config()
            if scenario == 'retrieval_timeout':
                p['gantry_motion']['timeout_s'] = .06
            if scenario == 'reference_timeout':
                p['exit_motion'].update(timeout_s=1.5, kp=.0001)
            config=Path(temp.name)/'coarse.yaml'
            config.write_text(yaml.safe_dump({'coarse':p},allow_unicode=True))
            bridge=CBoardBridgeNode(parameter_overrides=[Parameter('serial_port',value=board.path)])
            coarse=CoarseNode(parameter_overrides=[Parameter('config',value=str(config))],vision=VisionFixture(board))
            coarse.vision.owner=coarse
            coarse.vision.skip_exit=scenario == 'exit_timeout'
            coarse.vision.stall_ring_alignment=scenario == 'ring_timeout'
            # Four nodes share one Python executor in this software fixture.
            mission=MissionNode(parameter_overrides=[Parameter('route_control_timeout_s', value=2.)])
            # Other area executors are outside this test; seed mission's already-accepted
            # coarse-stage context, retaining its real speed/result validation callbacks.
            mission.timer.cancel()
            mission.state_timer.cancel()
            mission._snapshots_ready=lambda:True
            client=rclpy.create_node('coarse_pty_client')
            action=ActionClient(client,RunWorkBatch,'/vision/coarse/run_batch')
            executor=CommandExecutor(bridge._command_sub)
            for node in (bridge,coarse,mission,client):
                executor.add_node(node)
            board.thread.start()

            observed_states = set()
            def pump_until(predicate,timeout=6.):
                until=time.monotonic()+timeout
                while not predicate() and time.monotonic()<until:
                    mission.heartbeat.publish(String(data=mission.ctx.run_id))
                    executor.spin_once(timeout_sec=.005)
                    if coarse.task:
                        observed_states.add(coarse.task.state)
                self.assertTrue(predicate(),f'timeout: coarse={coarse.task.state if coarse.task else None}; '
                                f'mission={mission.ctx.state}:{mission.ctx.reason}; arm={bridge._arm.state}:{bridge._arm.reason}')

            pump_until(lambda:coarse._healthy() and coarse._idle() and mission._chassis_ready() and action.server_is_ready()
                       and all(pub.get_subscription_count() for pub in coarse._control_pubs.values()))
            req=ResetExecutor.Request(expected_run_id='',run_id=mission.ctx.run_id,reset_id='test-reset',
                                      home_node=1,initial_heading='DOWN')
            reset=coarse.contract.reset(req,ResetExecutor.Response())
            self.assertTrue(reset.accepted,reset.reason)
            for batch in ((1,2) if scenario in ('two_batches', 'deferred_exit') else (1,)):
                task_id=f'coarse-pty-{batch}'
                stage=Stage.COARSE_BATCH_1 if batch==1 else Stage.COARSE_BATCH_2
                mission.ctx.state='RUNNING'
                mission.ctx.stage=stage
                mission.ctx.stage_started_at=time.monotonic()
                mission.ctx.active=ActiveTask(task_id,'work','work',accepted=True,batch=batch,
                                             defer_reference_return=scenario == 'deferred_exit',
                                             spec=expected_steps('COARSE_PROCESS',[5,1,6],[2,3,1]))
                mission.sessions['work']=coarse.contract.session_id
                mission._route_control_sequence=0
                mission._route_control_wait_started=time.monotonic()
                goal=RunWorkBatch.Goal(run_id=mission.ctx.run_id,task_id=task_id,task_revision=1,
                                       stage=int(stage),batch=batch,operation='COARSE_PROCESS',
                                       colors=[5,1,6],positions=[2,3,1],enable_motion=True,
                                       defer_reference_return=scenario == 'deferred_exit')
                observed_states.clear()
                send=action.send_goal_async(goal)
                pump_until(send.done)
                handle=send.result()
                self.assertTrue(handle.accepted)
                result=handle.get_result_async()
                if scenario == 'serial_pause':
                    for phase in ('ALIGN_RING', 'PLACE'):
                        pump_until(lambda: coarse.task.state == phase and
                                   (phase != 'PLACE' or bridge._arm.state == 'MOVING'))
                        task, binding = coarse.task, coarse._binding
                        board.feedback_enabled = False
                        pump_until(lambda: getattr(coarse, '_link_paused_at', None) is not None)
                        self.assertIs(coarse.task, task)
                        self.assertFalse(task.finished)
                        self.assertEqual(mission.ctx.state, 'RUNNING', mission.ctx.reason)
                        self.assertEqual(task.velocity, (0., 0., 0.))
                        board.feedback_enabled = True
                        pump_until(lambda: getattr(coarse, '_link_paused_at', None) is None)
                        self.assertEqual(coarse._binding, binding)
                if scenario == 'cancel_servo':
                    pump_until(lambda:coarse.task.state == 'ALIGN_RING' and bridge._arm.state == 'SERVO')
                    cancel=handle.cancel_goal_async()
                    pump_until(cancel.done)
                    self.assertTrue(cancel.result().goals_canceling)
                pump_until(result.done,timeout=45. if scenario == 'ring_timeout' else 25.)
                response=result.result()
                if scenario in ('cancel_servo','reject_place','invalid_position'):
                    self.assertEqual(response.result.outcome,'STOPPED' if scenario == 'cancel_servo' else 'FAILED')
                    self.assertFalse(response.result.all_items_completed)
                    self.assertFalse(response.result.travel_ready)
                    if scenario == 'reject_place':
                        self.assertIn('rejected: 5',response.result.reason)
                    if scenario == 'invalid_position':
                        self.assertIn('位置反馈无效',response.result.reason)
                        self.assertEqual(coarse.task.placements, {})
                    mission._fault(response.result.reason)
                    pump_until(lambda:bridge._arm.idle and not bridge._arm.pending)
                    break
                self.assertEqual(response.result.outcome,'COMPLETED',
                                 f'{response.result.reason}; board={board.command}, error={board.error}; '
                                 f'bridge={bridge._adapter.command_reason}; mission={mission.ctx.state}:{mission.ctx.reason}')
                if scenario == 'ring_timeout':
                    self.assertIn('圆环PID达到5秒限时', response.result.reason)
                    self.assertIn('未确认达标', response.result.reason)
                self.assertEqual(len(response.result.confirmed_steps),6)
                self.assertEqual(response.result.final_alignment_outcome,
                                 'TIMEOUT_SKIPPED' if scenario == 'exit_timeout' else 'ALIGNED')
                self.assertEqual(response.result.reference_restored, scenario not in ('deferred_exit', 'reference_timeout'))
                self.assertEqual(response.result.reference_return_deferred, scenario == 'deferred_exit')
                self.assertEqual(response.result.reference_return_timed_out, scenario == 'reference_timeout')
                if scenario == 'retrieval_timeout':
                    self.assertTrue(coarse.task.return_timed_out)
                self.assertTrue(response.result.travel_ready)
                samples=board.arm_actions[-9:]
                placed=[(fb,yaw) for cmd,fb,yaw in samples if cmd==6]
                picked=[(fb,yaw) for cmd,fb,yaw in samples if cmd==5]
                self.assertEqual(len(placed),3)
                if scenario not in ('retrieval_timeout', 'serial_pause'):
                    for actual,target in zip(picked,placed):
                        self.assertLessEqual(abs(actual[0]-target[0]),.06)
                        self.assertLessEqual(abs(actual[1]-target[1]),.006)
                if scenario == 'deferred_exit':
                    self.assertIn('STOP_EXIT', observed_states)
                    self.assertNotIn('RETURN_REFERENCE', observed_states)
                    if batch == 1:
                        self.assertGreater(math.hypot(coarse.odom.x_m-coarse.task.reference[0],
                                                     coarse.odom.y_m-coarse.task.reference[1]), .003)
                elif scenario != 'reference_timeout':
                    self.assertAlmostEqual(coarse.odom.x_m, coarse.task.reference[0], delta=.0015)
                    self.assertAlmostEqual(coarse.odom.y_m, coarse.task.reference[1], delta=.0015)
                self.assertEqual(coarse.task.velocity,(0.,0.,0.))
                mission._action_result(mission.ctx.active,response)
                self.assertEqual(mission.ctx.stage,Stage.MOVE_TO_STORAGE_1 if batch==1 else Stage.MOVE_TO_STORAGE_2)
                self.assertEqual(mission.ctx.state,'RUNNING',mission.ctx.reason)
                pump_until(lambda:coarse.goal is None)
            commands=[payload[0] for _,mid,payload in board.frames if mid==ARM_CMD]
            if scenario in ('cancel_servo', 'invalid_position'):
                self.assertEqual(commands,[3,0])
            elif scenario == 'reject_place':
                self.assertEqual(commands,[3,6,0])
            elif scenario == 'serial_pause':
                self.assertEqual(commands,[3,6,3,6,6,6,5,5,5,3,2])
            else:
                self.assertEqual(commands,[3,6,6,6,5,5,5,3,2]*(2 if scenario in ('two_batches','deferred_exit') else 1))
            jogs=[(t,JOG_STRUCT.unpack(payload)) for t,mid,payload in board.frames if mid==ARM_JOG]
            self.assertTrue(jogs)
            self.assertTrue(all(jog[2]==0 for _,jog in jogs))
            self.assertEqual(jogs[-1][1][3],0)
            if scenario in ('two_batches','exit_timeout'):
                self.assertTrue(any(mid==CHASSIS_CMD and CHASSIS_STRUCT.unpack(payload)[3]==1
                                    for _,mid,payload in board.frames))
            self.assertIsNone(board.error)
        finally:
            if action:
                action.destroy()
            if mission:
                for action_client in (*mission.task_clients.work.values(), mission.task_clients.scan):
                    if action_client is not None:
                        action_client.destroy()
            if executor:
                for node in (coarse,mission,bridge,client):
                    if node:
                        executor.remove_node(node)
                        node.destroy_node()
                executor.shutdown()
            board.close()
            temp.cleanup()
            rclpy.shutdown()
            sys.setswitchinterval(switch_interval)


if __name__=='__main__':
    unittest.main()
