"""Opt-in real raw/mission/bridge integration; only an explicit PTY and software scene."""

import math
import os
from pathlib import Path
import tempfile
import sys
import time
import unittest

import yaml

from agv_perception.work_control import Observation
from agv_perception.raw_control import RawTask
from agv_perception.coarse_control import read_coarse_config
from test_coarse_ros_pty import ArmBoardPeer
from test_raw_control import calibrated_raw_config


def raw_scene(board):
    """Top-down moving camera: rail pivots, extends and carries camera orientation.

    Pixel-plane world is right/down. Positive yaw rotates counterclockwise;
    positive extension moves the camera image-up. FB gain is a fixture unit scale.
    """
    yaw = board.arm_yaw
    c, s = math.cos(yaw), math.sin(yaw)
    r = 100.+board.arm_fb*15.
    cam_x, cam_y = -r*s-board.x*200., 100.-r*c+board.y*200.
    points = {}
    for color, angle in ((1,0.),(3,2*math.pi/3),(4,4*math.pi/3)):
        x, y = 12.*math.cos(angle)-cam_x,12.*math.sin(angle)-cam_y
        points[color] = (100.+c*x-s*y,100.+s*x+c*y)
    return points


class RawVisionFixture:
    ready = True
    error = ''

    def __init__(self, board, scenario):
        self.board, self.scenario = board, scenario
        self.active = False
        self.kind = ''
        self.owner = None
        self.sequence = 0
        self.packet = None
        self.at = 0.
        self.started = None
        self.requests = []

    def request(self, active, kind='', color=0, area='coarse', *, geometry_target=0):
        self.active, self.kind = active, kind
        self.requests.append((active, kind, color, area))

    def latest(self):
        now = time.monotonic()
        if not self.active or not self.kind:
            self.packet = None
            self.started = None
            return None
        if self.started is None:
            self.started = now
        if now-self.at < .035:
            return self.packet
        self.at = now
        self.sequence += 1
        points = raw_scene(self.board)
        if self.scenario == 'missing':
            points = {}
        self.packet = Observation('material', self.sequence, now, (201, 201), {}, tuple(points.items()))
        return self.packet

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_RAW_ROS_TEST') == '1', 'opt in to isolated ROS + PTY integration')
class RawRosTests(unittest.TestCase):
    def test_two_batches_real_mission_and_same_grasp_action(self):
        self.run_scenario('two_batches')

    def test_deferred_exit_hands_offset_to_navigation(self):
        self.run_scenario('deferred_exit')

    def test_cancel_while_grasp_pending_sends_abort_and_no_next_pick(self):
        self.run_scenario('cancel_pick')

    def test_pick_rejection_stops_without_retry(self):
        self.run_scenario('reject_pick')

    def test_no_material_waits_after_base_approach_until_cancel(self):
        self.run_scenario('missing')

    def run_scenario(self, scenario):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT
        from agv_interfaces.action import RunWorkBatch
        from agv_interfaces.msg import RouteControl
        from agv_interfaces.srv import ResetExecutor
        from agv_mission.context import ActiveTask
        from agv_mission.mission_node import MissionNode
        from agv_mission.stages import Stage, expected_steps
        from agv_perception.coarse_node import CoarseNode
        from std_msgs.msg import String

        domain = int(os.environ.get('ROS_DOMAIN_ID', '171')) + (
            'two_batches', 'cancel_pick', 'reject_pick', 'missing', 'deferred_exit').index(scenario)
        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)
        rclpy.init(domain_id=domain)
        board = ArmBoardPeer()
        board.reject_cmd = 4 if scenario == 'reject_pick' else None
        nodes, actions = [], []
        executor = None
        board.thread.start()
        with tempfile.TemporaryDirectory(prefix='raw_pty_') as temp:
            try:
                p = calibrated_raw_config()
                p['motion_prediction']['enabled'] = True
                p['grasp_gate'].update(tolerance_px=.8,small_yaw_rad_s=.01,small_fb_rpm=.25)
                p['execution']['feedback_timeout_s'] = .5
                p['gantry_motion'].update(max_yaw_rad_s=.05,max_fb_rpm=2.,angle_tolerance_rad=.001,length_tolerance_pulses=1.)
                coarse_config = Path(temp)/'coarse.yaml'
                coarse_config.write_text(yaml.safe_dump({'coarse': read_coarse_config(Path(__file__).parents[1]/'config/coarse.yaml')}))
                raw_config = Path(temp)/'raw.yaml'
                raw_config.write_text(yaml.safe_dump({'raw': p}))
                bridge = CBoardBridgeNode(parameter_overrides=[Parameter('serial_port', value=board.path)])
                nodes.append(bridge)
                scene = RawVisionFixture(board, scenario)
                host = CoarseNode(parameter_overrides=[Parameter('config', value=str(coarse_config)),
                                                       Parameter('raw_config', value=str(raw_config))], vision=scene)
                nodes.append(host)
                scene.owner = host
                mission = MissionNode()
                nodes.append(mission)
                mission.timer.cancel()
                mission.state_timer.cancel()
                mission._snapshots_ready = lambda: True
                client = rclpy.create_node('raw_pty_client')
                nodes.append(client)
                action = ActionClient(client, RunWorkBatch, '/vision/raw/run_batch')
                other = ActionClient(client, RunWorkBatch, '/vision/coarse/run_batch')
                actions.extend((action, other))
                controls = []
                def observe_control(msg):
                    stamp = msg.header.stamp
                    age = (client.get_clock().now().nanoseconds-stamp.sec*1_000_000_000-stamp.nanosec)/1e9
                    # A queued tracking sample can arrive after the task enters PICK.
                    if host.task and time.monotonic()-age >= host.task.entered_at:
                        controls.append((host.task.state, msg.command))
                client.create_subscription(RouteControl, '/vision/raw/control', observe_control, 100)
                executor = CommandExecutor(bridge._command_sub)
                for node in nodes:
                    executor.add_node(node)

                def pump_until(predicate, timeout=6.):
                    until = time.monotonic()+timeout
                    while not predicate() and time.monotonic() < until:
                        mission.heartbeat.publish(String(data=mission.ctx.run_id))
                        executor.spin_once(timeout_sec=.005)
                    self.assertTrue(predicate(), f'{scenario}: host={host.task.state if host.task else None}; '
                                    f'mission={mission.ctx.state}:{mission.ctx.reason}; '
                                    f'arm={bridge._arm.state}:{bridge._arm.reason}; peer_error={board.error}')

                pump_until(lambda: host._healthy() and host._idle() and mission._chassis_ready() and action.server_is_ready())
                reset = host.contract.reset(ResetExecutor.Request(expected_run_id='', run_id=mission.ctx.run_id,
                                             reset_id='raw-reset', home_node=1, initial_heading='DOWN'), ResetExecutor.Response())
                self.assertTrue(reset.accepted, reset.reason)
                references = []
                for batch in ((1, 2) if scenario == 'two_batches' else (1,)):
                    task_id = f'raw-pty-{batch}'
                    stage = Stage.RAW_BATCH_1 if batch == 1 else Stage.RAW_BATCH_2
                    mission.ctx.state = 'RUNNING'
                    mission.ctx.stage = stage
                    mission.ctx.active = ActiveTask(task_id, 'work', 'work', accepted=True, batch=batch,
                                                    spec=expected_steps('RAW_PICK', [1, 3, 4], []),
                                                    defer_reference_return=scenario == 'deferred_exit')
                    mission.sessions['work'] = host.contract.session_id
                    mission._route_control_sequence = 0
                    mission._route_control_wait_started = time.monotonic()
                    goal = RunWorkBatch.Goal(run_id=mission.ctx.run_id, task_id=task_id, task_revision=1,
                                            stage=int(stage), batch=batch, operation='RAW_PICK',
                                            colors=[1, 3, 4], enable_motion=True,
                                            defer_reference_return=scenario == 'deferred_exit')
                    send = action.send_goal_async(goal)
                    pump_until(send.done)
                    handle = send.result()
                    self.assertTrue(handle.accepted)
                    result = handle.get_result_async()
                    pump_until(lambda: host.task is not None and host.task.state != 'ENTER_WORK_POSE')
                    
                    # Both area Actions share one reservation/arm/camera owner.
                    competing = RunWorkBatch.Goal(run_id=mission.ctx.run_id, task_id='competing', task_revision=1,
                                                  stage=6, batch=1, operation='COARSE_PROCESS',
                                                  colors=[1, 3, 4], positions=[1, 2, 3], enable_motion=True)
                    conflict = other.send_goal_async(competing)
                    pump_until(conflict.done)
                    self.assertFalse(conflict.result().accepted)
                    if scenario == 'missing':
                        pump_until(lambda: host.task.state == 'WAIT_REQUIRED',timeout=10.)
                        started = time.monotonic()
                        pump_until(lambda: time.monotonic()-started > 1.2)
                        self.assertFalse(result.done())
                        self.assertEqual(host.task.state,'WAIT_REQUIRED')
                        cancel = handle.cancel_goal_async()
                        pump_until(cancel.done)
                    if scenario == 'cancel_pick':
                        pump_until(lambda: host.task.state == 'PICK' and bridge._arm.pending and
                                   any(mid == ARM_CMD and data[0] == 4 for _,mid,data in board.frames))
                        cancel = handle.cancel_goal_async()
                        pump_until(cancel.done)
                        self.assertTrue(cancel.result().goals_canceling)
                    pump_until(result.done, timeout=22.)
                    response = result.result()
                    r = response.result
                    if scenario not in ('two_batches', 'deferred_exit'):
                        self.assertEqual(r.outcome, 'STOPPED' if scenario in ('cancel_pick','missing') else 'FAILED')
                        self.assertFalse(r.travel_ready)
                        self.assertFalse(r.all_items_completed)
                        self.assertEqual(r.confirmed_steps, [])
                        mission._fault(r.reason)
                        pump_until(lambda: bridge._arm.idle and not bridge._arm.pending)
                        break
                    self.assertIsNone(board.error)
                    self.assertEqual(r.outcome, 'COMPLETED', r.reason)
                    self.assertEqual(r.reference_restored, scenario != 'deferred_exit')
                    self.assertEqual(r.reference_return_deferred, scenario == 'deferred_exit')
                    self.assertFalse(r.reference_return_timed_out)
                    if scenario == 'deferred_exit':
                        self.assertGreater(board.x-host.task.reference[0], .010)
                    self.assertTrue(r.arm_idle and r.motion_released and r.travel_ready)
                    references.append(host.task.reference)
                    self.assertEqual(r.final_alignment_outcome, 'NOT_RUN')
                    self.assertEqual([(s.step_id, s.color, s.position) for s in r.confirmed_steps],
                                     [('PICK:1', 1, 0), ('PICK:2', 3, 0), ('PICK:3', 4, 0)])
                    mission._action_result(mission.ctx.active, response)
                    self.assertEqual(mission.ctx.stage, Stage.MOVE_TO_COARSE_1 if batch == 1 else Stage.MOVE_TO_COARSE_2)
                    self.assertEqual(mission.ctx.state, 'RUNNING', mission.ctx.reason)
                    pump_until(lambda: host.goal is None)
                    if batch == 1:
                        # Establish a different second-entry pose in the software peer.
                        board.x, board.y = .15, .02
                        previous = host.odom.x_m
                        pump_until(lambda: abs(host.odom.x_m-previous) > .05)
                commands = [payload[0] for _, mid, payload in board.frames if mid == ARM_CMD]
                if scenario == 'two_batches':
                    self.assertEqual(commands, [3, 4, 4, 4, 2]*2)
                    self.assertNotEqual(references[0], references[1])
                    self.assertTrue(any(mid == CHASSIS_CMD and any(CHASSIS_STRUCT.unpack(payload)[:3])
                                        for _, mid, payload in board.frames))
                elif scenario == 'deferred_exit':
                    self.assertEqual(commands, [3, 4, 4, 4, 2])
                elif scenario == 'missing':
                    self.assertEqual(commands, [3, 0])
                else:
                    self.assertEqual(commands, [3, 4, 0])
                if scenario != 'missing':
                    from agv_cboard.protocol import JOG_STRUCT
                    jogs = [JOG_STRUCT.unpack(payload) for _,mid,payload in board.frames if mid == ARM_JOG]
                    self.assertTrue(any(abs(j[0]) > 0 and abs(j[1]) > 0 and j[3] == 1 for j in jogs))
                    self.assertEqual(jogs[-1][3],0)
                # Fixed arm programs retain zero base commands.
                for i,(at,mid,payload) in enumerate(board.frames):
                    if mid == ARM_CMD and payload[0] == 2:
                        for later,other,data in board.frames[i+1:]:
                            if later-at >= .12:
                                break
                            if other == CHASSIS_CMD:
                                self.assertEqual(CHASSIS_STRUCT.unpack(data)[:2],(0.,0.))
                for state, cmd in controls:
                    if state in RawTask.ARM_STATES:
                        self.assertEqual((cmd.vx_mps, cmd.vy_mps, cmd.omega_deg_s), (0., 0., 0.))
                self.assertTrue(all(color == 0 and area == 'raw' for active, kind, color, area in scene.requests
                                    if active and kind == 'material'))
                self.assertIsNone(board.error)
            finally:
                for action in actions:
                    action.destroy()
                for node in reversed(nodes):
                    if executor:
                        executor.remove_node(node)
                    node.destroy_node()
                if executor:
                    executor.shutdown()
                board.close()
                rclpy.shutdown()
                sys.setswitchinterval(switch_interval)



if __name__ == '__main__':
    unittest.main()
