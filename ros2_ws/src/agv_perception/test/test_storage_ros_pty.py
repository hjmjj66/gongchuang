"""Opt-in shared vision/mission/serial integration, with explicit software perception."""

import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

import yaml

from agv_perception.coarse_control import Observation
from agv_perception.coarse_control import read_coarse_config
from test_coarse_ros_pty import ArmBoardPeer, coarse_scene, fast_coarse_config
from test_storage_control import calibrated_storage_config
from test_raw_control import calibrated_raw_config


def fast_storage_config():
    from copy import deepcopy
    p=deepcopy(fast_coarse_config())
    p['arm'].pop('pick_cmd')
    p['arm'].update(place_cmd=6,stack_cmd=7)
    p['ring_search'].update(yaw_rad_s=.4,duration_s=1.,scan_settle_frames=2)
    return p


class StorageVisionFixture:
    ready = True
    error = ''

    def __init__(self, board, scenario):
        self.board, self.scenario = board, scenario
        self.owner = None
        self.active = False
        self.kind = ''
        self.sequence = 0

    def request(self, active, kind='', color=0, area='coarse', *, geometry_target=0):
        self.active, self.kind = active, kind
        self.color = color
        self.geometry_target = geometry_target

    def latest(self):
        if not self.active or not self.kind:
            return None
        self.sequence += 1
        rings = coarse_scene(self.board)
        task = self.owner.task
        mapping = (tuple(zip(task.colors, task.positions)) if task and
                   getattr(task, 'operation', 'COARSE_PROCESS') == 'COARSE_PROCESS' else ((1, 1), (5, 2), (6, 3)))
        materials = tuple((color, rings[position]) for color, position in mapping)
        if self.scenario == 'entry_timeout' and task and task.state == 'ALIGN_ENTRY':
            rings = {}
        if task and task.state == 'SCAN_FIRST_LAYER' and self.scenario in ('two_visits', 'cancel_scan'):
            materials = tuple((c,p) for c,p in materials if abs(p[0]-100.) < 9.)
            if self.scenario == 'cancel_scan':
                materials = tuple((c,p) for c,p in materials if c == 5)
        return Observation(self.kind, self.sequence, time.monotonic(), (201, 201),
                           rings if self.kind == 'ring' else {}, materials if self.kind == 'material' else (),
                           geometry_target=self.geometry_target)

    def close(self):
        pass


@unittest.skipUnless(os.environ.get('AGV_STORAGE_ROS_TEST') == '1', 'opt in to isolated ROS + PTY integration')
class StorageRosTests(unittest.TestCase):
    def test_first_visit_stows_and_handoffs_without_returning_p0(self):
        self.run_scenario('deferred_exit')

    def test_two_visits_partial_scan_and_shared_coarse_executor(self):
        self.run_scenario('two_visits')

    def test_first_entry_angle_timeout_still_calibrates_places_and_stows(self):
        self.run_scenario('entry_timeout')

    def test_stack_rejection_is_not_retried(self):
        self.run_scenario('reject_stack')

    def test_cancel_scan_stops_without_stack_action(self):
        self.run_scenario('cancel_scan')

    def test_cancel_during_first_exit_stops_and_aborts_before_stage_advance(self):
        self.run_scenario('cancel_exit')

    def test_storage_two_visits_independent_of_coarse_rework(self):
        self.run_scenario('two_visits', include_coarse=False)

    def run_scenario(self, scenario, include_coarse=True):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from agv_cboard.cboard_bridge_node import CBoardBridgeNode
        from agv_cboard.command_executor import CommandExecutor
        from agv_cboard.protocol import ARM_CMD, ARM_JOG, CHASSIS_CMD, CHASSIS_STRUCT, JOG_STRUCT
        from agv_interfaces.action import RunWorkBatch
        from agv_interfaces.msg import FirstLayerRecord
        from agv_interfaces.srv import ResetExecutor
        from agv_mission.context import ActiveTask
        from agv_mission.mission_node import MissionNode
        from agv_mission.stages import Stage, expected_steps
        from agv_mission.task_code import parse_task_code
        from agv_perception.coarse_node import CoarseNode
        from std_msgs.msg import String

        domain = int(os.environ.get('ROS_DOMAIN_ID', '151')) + (
            'two_visits', 'entry_timeout', 'reject_stack', 'cancel_scan', 'cancel_exit', 'deferred_exit').index(scenario)
        rclpy.init(domain_id=domain)
        # Four ROS nodes and the PTY peer share this interpreter; do not starve its 20 ms serial writer.
        switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(.001)
        board = ArmBoardPeer()
        board.x,board.y,board.yaw = .012,-.012,.08
        board.reject_cmd = 7 if scenario == 'reject_stack' else None
        nodes, actions = [], []
        executor = None
        with tempfile.TemporaryDirectory(prefix='storage_pty_') as temp:
            try:
                coarse_p = fast_coarse_config()
                storage_p = fast_storage_config()
                coarse_path, storage_path = Path(temp)/'coarse.yaml', Path(temp)/'storage.yaml'
                raw_path = Path(temp)/'raw.yaml'
                raw_path.write_text(yaml.safe_dump({'raw': calibrated_raw_config()}))
                coarse_path.write_text(yaml.safe_dump({'coarse': coarse_p}))
                storage_path.write_text(yaml.safe_dump({'storage': storage_p}))
                bridge = CBoardBridgeNode(parameter_overrides=[Parameter('serial_port', value=board.path)])
                nodes.append(bridge)
                fixture = StorageVisionFixture(board, scenario)
                work = CoarseNode(vision=fixture, parameter_overrides=[
                    Parameter('config', value=str(coarse_path)), Parameter('storage_config', value=str(storage_path)),
                    Parameter('raw_config', value=str(raw_path))])
                nodes.append(work)
                fixture.owner = work
                mission = MissionNode()
                nodes.append(mission)
                mission.timer.cancel()
                mission.state_timer.cancel()
                mission._snapshots_ready = lambda: True
                mission.ctx.code = parse_task_code('156+123+516+231')
                client = rclpy.create_node('storage_pty_client')
                nodes.append(client)
                storage_action = ActionClient(client, RunWorkBatch, '/vision/storage/run_batch')
                coarse_action = ActionClient(client, RunWorkBatch, '/vision/coarse/run_batch')
                actions.extend((storage_action, coarse_action))
                executor = CommandExecutor(bridge._command_sub)
                for node in nodes:
                    executor.add_node(node)
                board.thread.start()

                def pump(predicate, timeout=7.):
                    until = time.monotonic()+timeout
                    while not predicate() and time.monotonic() < until:
                        mission.heartbeat.publish(String(data=mission.ctx.run_id))
                        executor.spin_once(timeout_sec=.005)
                    self.assertTrue(predicate(), f'{work.task.state if work.task else None}; '
                                    f'{mission.ctx.state}: {mission.ctx.reason}; {bridge._arm.reason}')

                pump(lambda: work._healthy() and work._idle() and mission._chassis_ready()
                     and all(a.server_is_ready() for a in actions)
                     and all(pub.get_subscription_count() for pub in work._control_pubs.values()))
                response = work.contract.reset(ResetExecutor.Request(expected_run_id='', run_id=mission.ctx.run_id,
                    reset_id='storage-reset', home_node=1, initial_heading='DOWN'), ResetExecutor.Response())
                self.assertTrue(response.accepted, response.reason)
                self.assertTrue(work.contract.snapshot().config_ready)
                stages = (([Stage.STORAGE_PLACE_1, Stage.COARSE_BATCH_2, Stage.STORAGE_STACK_2] if include_coarse
                           else [Stage.STORAGE_PLACE_1, Stage.STORAGE_STACK_2])
                          if scenario == 'two_visits' else
                          [Stage.STORAGE_PLACE_1] if scenario in ('entry_timeout','cancel_exit','deferred_exit') else [Stage.STORAGE_STACK_2])
                for index, stage in enumerate(stages):
                    # Navigation between areas is outside this work-executor harness.
                    board.x,board.y,board.yaw=.012,-.012,.08
                    pump(lambda:abs(work.odom.x_m)<.001 and abs(work.odom.y_m)<.001)
                    is_coarse = stage == Stage.COARSE_BATCH_2
                    is_place = stage == Stage.STORAGE_PLACE_1
                    operation = 'COARSE_PROCESS' if is_coarse else 'STORAGE_PLACE' if is_place else 'STORAGE_STACK'
                    colors = [1, 5, 6] if is_place else [5, 1, 6]
                    positions = [1, 2, 3] if is_place else [2, 3, 1] if is_coarse else [2, 1, 3]
                    batch = 1 if is_place else 2
                    task_id = f'storage-pty-{index}'
                    mission.ctx.state, mission.ctx.stage = 'RUNNING', stage
                    mission.ctx.stage_started_at = time.monotonic()
                    mission.ctx.active = ActiveTask(task_id, 'work', 'work', accepted=True, batch=batch,
                                                    defer_reference_return=scenario == 'deferred_exit',
                                                    spec=expected_steps(operation, colors, positions))
                    mission.sessions['work'] = work.contract.session_id
                    mission._route_control_sequence = 0
                    mission._route_control_wait_started = time.monotonic()
                    goal = RunWorkBatch.Goal(run_id=mission.ctx.run_id, task_id=task_id, task_revision=1,
                        stage=int(stage), batch=batch, operation=operation, colors=colors, positions=positions,
                        first_layer=([FirstLayerRecord(color=c, position=p) for c, p in ((1, 1), (5, 2), (6, 3))]
                                     if operation == 'STORAGE_STACK' else []), enable_motion=True,
                        defer_reference_return=scenario == 'deferred_exit')
                    action = coarse_action if is_coarse else storage_action
                    if operation == 'STORAGE_STACK':
                        goal.positions = [2, 3, 1]
                        invalid = action.send_goal_async(goal)
                        pump(invalid.done)
                        self.assertFalse(invalid.result().accepted)
                        goal.positions = positions
                    start_xy = board.x, board.y
                    frame_start = len(board.frames)
                    send = action.send_goal_async(goal)
                    pump(send.done)
                    handle = send.result()
                    self.assertTrue(handle.accepted)
                    result = handle.get_result_async()
                    # Both endpoints share a single reservation/runtime owner.
                    conflict_goal = RunWorkBatch.Goal(run_id=mission.ctx.run_id, task_id='conflicting-task',
                        task_revision=1, stage=8 if is_coarse else 12, batch=1 if is_coarse else 2,
                        operation='STORAGE_PLACE' if is_coarse else 'COARSE_PROCESS',
                        colors=[1, 5, 6] if is_coarse else [5, 1, 6],
                        positions=[1, 2, 3] if is_coarse else [2, 3, 1], enable_motion=True)
                    conflict = (coarse_action if not is_coarse else storage_action).send_goal_async(conflict_goal)
                    pump(conflict.done)
                    self.assertFalse(conflict.result().accepted)
                    if scenario in ('cancel_scan','cancel_exit'):
                        pump(lambda: (work.task.state == 'SCAN_FIRST_LAYER' and time.monotonic()-work.task.entered_at > 2.)
                             if scenario == 'cancel_scan' else (work.task.state == 'RETURN_REFERENCE' and math.hypot(*work.task.velocity[:2]) > 0.), timeout=15.)
                        cancel = handle.cancel_goal_async()
                        pump(cancel.done)
                        self.assertTrue(cancel.result().goals_canceling)
                    pump(result.done, 30.)
                    response = result.result()
                    commands = [payload[0] for _, mid, payload in board.frames[frame_start:] if mid == ARM_CMD]
                    if scenario in ('reject_stack', 'cancel_scan', 'cancel_exit'):
                        self.assertEqual(response.result.outcome, 'FAILED' if scenario == 'reject_stack' else 'STOPPED')
                        self.assertEqual(response.result.all_items_completed,scenario == 'cancel_exit')
                        pump(lambda: any(mid == ARM_CMD and payload[0] == 0
                                         for _, mid, payload in board.frames[frame_start:]))
                        commands = [payload[0] for _, mid, payload in board.frames[frame_start:] if mid == ARM_CMD]
                        self.assertEqual(commands, [3, 3, 7, 0] if scenario == 'reject_stack' else [3,6,6,6,2,0] if scenario == 'cancel_exit' else [3, 0])
                        pump(lambda: bridge._arm.idle and not bridge._arm.pending)
                        break
                    self.assertEqual(response.result.outcome, 'COMPLETED', response.result.reason)
                    self.assertEqual(len(response.result.confirmed_steps), 6 if is_coarse else 3)
                    self.assertEqual(response.result.reference_restored,(is_coarse or is_place) and scenario != 'deferred_exit')
                    self.assertEqual(response.result.reference_return_deferred,scenario == 'deferred_exit')
                    self.assertTrue(response.result.travel_ready)
                    self.assertEqual(response.result.final_alignment_outcome, 'ALIGNED' if is_coarse else 'NOT_RUN')
                    if not is_coarse:
                        self.assertEqual(commands, [3, 6, 6, 6, 2] if is_place else [3, 3, 7, 7, 7, 2])
                        jogs=[JOG_STRUCT.unpack(payload) for _,mid,payload in board.frames[frame_start:] if mid == ARM_JOG]
                        self.assertTrue(jogs)
                        self.assertTrue(all(jog[2] == 0 for jog in jogs))
                        if is_place:
                            if scenario == 'deferred_exit':
                                self.assertGreater(math.hypot(work.odom.x_m-work.task.reference[0],
                                                             work.odom.y_m-work.task.reference[1]), .003)
                            else:
                                self.assertAlmostEqual(work.odom.x_m, work.task.reference[0], delta=.0015)
                                self.assertAlmostEqual(work.odom.y_m, work.task.reference[1], delta=.0015)
                            self.assertLessEqual(math.hypot(work.odom.vx_mps,work.odom.vy_mps),
                                                 storage_p['base_servo']['stop_speed_mps'])
                        else:
                            self.assertIsNone(work.task.reference)
                            self.assertAlmostEqual(board.x,start_xy[0],delta=.001)
                            self.assertAlmostEqual(board.y,start_xy[1],delta=.001)
                    if operation == 'STORAGE_STACK':
                        self.assertEqual([(r.color, r.position) for r in response.result.confirmed_steps],
                                         [(5, 2), (1, 1), (6, 3)])
                        self.assertTrue(any(mid == ARM_JOG for _, mid, _ in board.frames[frame_start:]))
                        self.assertFalse(any(mid == CHASSIS_CMD and any(abs(v)>1e-6 for v in CHASSIS_STRUCT.unpack(payload)[:3])
                                             for _,mid,payload in board.frames[frame_start:]))
                    mission._action_result(mission.ctx.active, response)
                    self.assertEqual(mission.ctx.state, 'RUNNING', mission.ctx.reason)
                    self.assertEqual(mission.ctx.stage, Stage.MOVE_TO_RAW_2 if is_place else
                                     Stage.MOVE_TO_STORAGE_2 if is_coarse else Stage.RETURN_HOME)
                    pump(lambda: work.goal is None)
                if scenario == 'two_visits':
                    self.assertEqual(mission.ctx.first_layer, {1: 1, 5: 2, 6: 3})
                    self.assertEqual(work.task.seen_colors, {1, 5, 6})
                if scenario == 'entry_timeout':
                    self.assertEqual(work.task.entry_alignment, 'TIMEOUT_SKIPPED')
                if scenario in ('two_visits','entry_timeout'):
                    self.assertTrue(any(mid == CHASSIS_CMD and CHASSIS_STRUCT.unpack(payload)[3]
                                        for _, mid, payload in board.frames))
                self.assertIsNone(board.error)
            finally:
                for action in actions:
                    action.destroy()
                for node in nodes:
                    if isinstance(node, MissionNode):
                        for action in (*node.task_clients.work.values(), node.task_clients.scan):
                            action.destroy()
                if executor:
                    for node in reversed(nodes):
                        executor.remove_node(node)
                        node.destroy_node()
                    executor.shutdown()
                if board.thread.ident is not None:
                    board.close()
                else:
                    os.close(board.master)
                    os.close(board.slave)
                rclpy.shutdown()
                sys.setswitchinterval(switch_interval)


if __name__ == '__main__':
    unittest.main()
