import copy
from pathlib import Path
import unittest

import yaml

from route_debug.session import Session, settings_from
from route_debug.map_view import measured_pose

CONFIG = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/debug.yaml').read_text())


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class IO:
    odom_timeout = .5

    def __init__(self):
        self.calls, self.commands, self.maps = [], [], []
        self.exclusive_ok = True

    def rpc(self, kind, payload, done):
        self.calls.append((kind, payload, done))

    def reply(self, kind, result):
        for i, (name, payload, done) in enumerate(self.calls):
            if name == kind:
                self.calls.pop(i)
                done(copy.deepcopy(result))
                return payload
        raise AssertionError('No call: ' + kind)

    def exclusive(self):
        return self.exclusive_ok

    def services_ready(self):
        return True

    def velocity(self, *values, binding=None):
        self.commands.append(values)

    def lease(self, run):
        pass

    def map(self, *args):
        self.maps.append(args)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.io, self.clock = IO(), Clock()
        self.s = Session(self.io, CONFIG, {}, self.clock)
        self.rt = dict(run_id='', executor_session_id='planner-1', busy=False,
                       quiescent=True, motion_released=True, source='REAL',
                       config_ready=True, map_ready=False, map_revision=0,
                       blocked_edges=[], odometry_ready=True, position_known=True,
                       heading_known=True, current_heading='DOWN', current_node=1,
                       last_task_id='', last_state='')
        self.refresh()

    def refresh(self):
        self.s.update_board(dict(connected=True, status_valid=True, pc_control=True,
                                 odometry_valid=True, motion_allowed=True, fault_reason='',
                                 odom_session_id='session-1',motion_epoch=1))
        self.s.update_odom(dict(x_m=0.,y_m=0.,yaw_rad=0.,yaw_deg=0.,vx_mps=0.,vy_mps=0.,
                                wz_rad_s=0.,valid=True,origin_initialized=True,
                                odom_session_id='session-1',motion_epoch=1))
        self.s.update_runtime(self.rt)
        self.s.heartbeat('browser-one')

    def begin(self, mode='free', start=1, goal=2, heading='DOWN', align=False):
        self.s.command('start', dict(mode=mode, start=start, goal=goal, heading=heading, align=align), 'browser-one')
        self.io.reply('get_runtime', {'runtime': self.rt})
        payload = self.io.calls[-1][1]
        self.assertEqual(payload['home_node'], start)
        self.rt.update(run_id=self.s.run_id, current_node=start, current_heading=self.s.settings['heading'])
        self.io.reply('reset_context', {'accepted': True, 'runtime': self.rt})
        self.refresh()
        self.s.tick()
        self.assertEqual(self.s.phase, 'WAIT_MAP')
        self.rt.update(map_ready=True, map_revision=1)
        self.refresh()
        self.s.tick()
        self.assertEqual(self.s.state, 'NAVIGATING')

    def plan(self, finish_early=False):
        request = next(c[1] for c in self.io.calls if c[0] == 'plan')
        end_heading = request['goal_heading'] or self.rt['current_heading']
        if finish_early:
            self.feedback('COMPLETED', sequence=1)
        self.io.reply('plan', {'plan': dict(status='ok', reason='', run_id=self.s.run_id,
            request_id=self.s.request_id, nodes=[self.rt['current_node'], request['goal']],
            end_heading=end_heading)})
        return request, end_heading

    def feedback(self, state='EXECUTING', sequence=1, **changes):
        msg = dict(run_id=self.s.run_id, request_id=self.s.request_id,
                   executor_session_id='planner-1', source='REAL', sequence=sequence,
                   state=state, command=dict(vx_mps=.5, vy_mps=0., omega_deg_s=0.,
                                            odom_session_id='session-1',motion_epoch=1))
        msg.update(changes)
        self.s.feedback(msg)

    def complete(self, finish_early=False):
        req, heading = self.plan(finish_early)
        if not finish_early:
            self.feedback('COMPLETED')
        self.rt.update(last_task_id=self.s.request_id, last_state='COMPLETED',
                       current_node=req['goal'], current_heading=heading)
        self.io.reply('get_runtime', {'runtime': self.rt})

    def test_arbitrary_initial_pose_and_terminal_before_response(self):
        self.begin(start=7, goal=9, heading='LEFT')
        self.complete(finish_early=True)
        self.assertEqual(self.s.state, 'FINISHED')
        self.assertTrue(all(cmd == (0., 0., 0.) for cmd in self.io.commands))

    def test_four_competition_combinations_and_exact_seven_waits(self):
        for start in (1, 3):
            for goal in (1, 3):
                with self.subTest(start=start, goal=goal):
                    self.setUp()
                    self.begin(mode='competition', start=start, goal=goal)
                    for index, target in enumerate([2, 4, 6, 8, 4, 6, 8, goal]):
                        req = next(c[1] for c in self.io.calls if c[0] == 'plan')
                        self.assertEqual(req['goal'], target)
                        self.assertEqual(req['align_at_goal'], index < 7)
                        self.complete()
                        if index < 7:
                            self.assertEqual(self.s.state, 'WAITING')
                            self.clock.now += 2.99
                            self.refresh()
                            self.s.tick()
                            self.assertEqual(self.s.state, 'WAITING')
                            self.clock.now += .02
                            self.refresh()
                            self.s.tick()
                            self.assertEqual(self.s.state, 'NAVIGATING')
                    self.assertEqual(self.s.state, 'FINISHED')

    def test_stopping_during_wait_never_dispatches_next_route(self):
        self.begin(mode='competition', goal=1)
        self.complete()
        self.s.stop('用户停止')
        self.io.reply('get_runtime', {'runtime': self.rt})
        self.assertEqual(self.s.state, 'STOPPED')
        self.clock.now += 5
        self.refresh()
        self.s.tick()
        self.assertFalse(any(c[0] == 'plan' for c in self.io.calls))
        self.s.command('start', dict(start=1, goal=2), 'browser-one')
        self.assertEqual(self.s.state,'INITIALIZING')

    def test_stale_feedback_and_duplicate_sequence_do_not_forward(self):
        self.begin()
        self.plan()
        self.feedback(sequence=1, run_id='old-run')
        self.feedback(sequence=1, executor_session_id='old-process')
        self.assertEqual(self.io.commands[-1], (0., 0., 0.))
        self.feedback(sequence=2)
        count = len(self.io.commands)
        self.feedback(sequence=2)
        self.feedback(sequence=1)
        self.assertEqual(len(self.io.commands), count)
        self.assertEqual(self.io.commands[-1], (.5, 0., 0.))
        self.clock.now += .31
        self.refresh()
        self.s.tick()
        self.assertEqual(self.s.state, 'STOPPING')
        self.assertEqual(self.io.commands[-1], (0., 0., 0.))

    def test_browser_loss_and_executor_restart_stop(self):
        for failure in ('browser', 'executor'):
            with self.subTest(failure=failure):
                self.setUp()
                self.begin()
                self.plan()
                if failure == 'browser':
                    self.clock.now += 8.1
                    self.refresh()
                    self.s.owner_at = 100.
                else:
                    self.rt['executor_session_id'] = 'new-process'
                    self.refresh()
                self.s.tick()
                self.assertEqual(self.s.state, 'STOPPING')

    def test_stop_while_plan_reply_pending_waits_before_stop_query(self):
        self.begin()
        self.s.stop('停止')
        self.assertEqual(self.s.phase, 'QUERY_STOP')
        self.rt.update(busy=True, active_task_id=self.s.request_id)
        self.io.reply('plan', {'plan': {}})  # Epoch-invalid response cannot start motion.
        self.s.tick()
        self.io.reply('get_runtime', {'runtime': self.rt})
        stop = next(c[1] for c in self.io.calls if c[0] == 'stop')
        self.assertEqual(stop['task_id'], self.s.request_id)
        self.assertFalse(self.s.accepted)

    def test_unresolved_move_does_not_claim_stopped_or_allow_reset(self):
        self.begin()
        self.clock.now += 5.1
        self.refresh()
        self.s.tick()
        self.clock.now += 10.1
        self.s.tick()
        self.assertEqual(self.s.state, 'FAULT')
        self.assertTrue(self.s.snapshot()['unresolved'])
        with self.assertRaises(ValueError):
            self.s.command('reset', dict(start=1, goal=2), 'browser-one')

    def test_session_change_cancels_and_start_reanchors_after_stop(self):
        self.begin()
        self.plan()
        self.s.update_board(dict(connected=True,status_valid=True,pc_control=True,
                                 odometry_valid=True,motion_allowed=True,
                                 odom_session_id='new-session',motion_epoch=3))
        self.assertEqual(self.s.state,'STOPPING')
        self.assertIsNone(self.s.anchor)
        self.assertEqual(list(self.s.trace),[])
        self.io.reply('get_runtime',{'runtime':self.rt})
        self.assertEqual(self.s.state,'FAULT')
        self.s.command('start',dict(start=7,goal=9,heading='LEFT'),'browser-one')
        self.assertEqual(self.s.state,'INITIALIZING')

    def test_completed_pose_is_not_snapped_to_map_endpoint(self):
        self.begin()
        self.s.pose=dict(x=900.,y=470.,angle=181.,valid=True)
        self.complete()
        self.assertEqual((self.s.pose['x'],self.s.pose['y']),(900.,470.))

    def test_future_sealing_rejects_nonempty_blocked_edges(self):
        with self.assertRaisesRegex(ValueError, '封路'):
            settings_from(dict(start=1, goal=2, blocked_edges=[[1,2]]))


class MapTests(unittest.TestCase):
    def test_measured_xy_not_snapped_to_road(self):
        anchor=dict(x_m=0.,y_m=0.,yaw_rad=0.,yaw_deg=0.,valid=True)
        odom=dict(anchor,x_m=.6,y_m=.2)
        p=measured_pose(odom,anchor,1,'DOWN',{})
        self.assertGreater(p['x'],821.)
        self.assertAlmostEqual(p['y'],141+(482-141)*.5)
        self.assertAlmostEqual(p['angle'],180.)

    def test_rotated_nonzero_anchor_and_yaw(self):
        import math
        a=dict(x_m=2.,y_m=3.,yaw_rad=math.pi/2,yaw_deg=90.,valid=True)
        p=measured_pose(dict(a,y_m=3.6,yaw_deg=135.),a,1,'DOWN',{})
        self.assertAlmostEqual(p['x'],821.)
        self.assertAlmostEqual(p['y'],141+(482-141)*.5)
        self.assertAlmostEqual(p['angle'],135.)

    def test_forward_left_display_for_all_map_headings(self):
        anchor=dict(x_m=0.,y_m=0.,yaw_rad=0.,yaw_deg=0.,valid=True)
        for heading, forward, left in (
                ('UP', (0,-1), (-1,0)), ('RIGHT', (1,0), (0,-1)),
                ('DOWN', (0,1), (1,0)), ('LEFT', (-1,0), (0,1))):
            for odom, expected in ((dict(anchor,x_m=.3),forward),
                                   (dict(anchor,y_m=.3),left)):
                with self.subTest(heading=heading,expected=expected):
                    p=measured_pose(odom,anchor,5,heading,{})
                    dx,dy=p['x']-500.,p['y']-482.
                    for actual,sign in ((dx,expected[0]),(dy,expected[1])):
                        if sign:
                            self.assertGreater(actual*sign,0.)
                        else:
                            self.assertAlmostEqual(actual,0.)


if __name__ == '__main__':
    unittest.main()
