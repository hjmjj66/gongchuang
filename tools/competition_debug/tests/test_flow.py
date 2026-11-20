"""Drive the actual full-run state machine with explicit executor responses."""

import itertools
from pathlib import Path
import importlib.util
import unittest

spec = importlib.util.spec_from_file_location('route_session_fixtures', Path(__file__).resolve().parents[2]/'route_debug/tests/test_session.py')
route_tests = importlib.util.module_from_spec(spec)
spec.loader.exec_module(route_tests)
from competition_debug.session import CompetitionSession, WORK_STAGE, run_options
from agv_mission.stages import batch_spec, expected_steps


class FullSessionTests(unittest.TestCase):
    refresh = route_tests.SessionTests.refresh
    plan = route_tests.SessionTests.plan
    feedback = route_tests.SessionTests.feedback
    complete = route_tests.SessionTests.complete

    def setUp(self):
        route_tests.SessionTests.setUp(self)
        self.io.config_display = {'mission_source': 'mission.yaml', 'use_lidar': False}
        self.io.use_lidar = False
        self.io.work_health = lambda active: ''
        self.io.release_camera = lambda: None
        self.io.fault_context = lambda: {'test': 'explicit software executor'}
        self.s = CompetitionSession(self.io, route_tests.CONFIG, {}, self.clock, parking_offset_mm=275)
        self.refresh()
        self.counts_shown = []
        self.io.show_counts = lambda *counts: self.counts_shown.append(counts)
        self.starts = []
        self.status = None

        def start(*args):
            self.starts.append(args)
        self.io.start_batch = start
        self.io.tick_work = lambda *args: self.status

    def begin(self, switches=None, start=1, goal=3, mode='web'):
        self.s.command('start', dict(mode=mode, start=start, goal=goal, real_actions=switches or
                       dict.fromkeys(('raw','coarse','storage'), True), blocked_edges=[[7,4]]), 'browser-one')
        if mode == 'screen':
            self.s.screen_message('ROUTE', (1,))
            self.s.screen_message('CFG', (start, goal))
            self.assertEqual(self.s.state, 'SCREEN_SETUP')
            self.s.screen_message('START', ())
            self.refresh()
            self.s.tick()
        self.io.reply('get_runtime', {'runtime': self.rt})
        self.rt.update(run_id=self.s.run_id, current_node=start, current_heading=self.s.settings['heading'])
        self.io.reply('reset_context', {'accepted': True, 'runtime': self.rt})
        self.refresh()
        self.s.tick()
        self.rt.update(map_ready=True, map_revision=1, blocked_edges=[dict(a=7,b=4)])
        self.refresh()
        self.s.tick()
        if mode == 'screen':
            self.assertNotEqual(self.s.state, 'NAVIGATING')
            self.clock.now += 2.
            self.refresh()
            self.s.tick()
        self.assertEqual(self.s.state, 'NAVIGATING', self.s.reason)

    def test_idle_camera_commands_do_not_start_run_and_are_rejected_during_run(self):
        camera = []
        self.io.open_idle_preview = lambda kind='main': camera.append(kind)
        self.io.release_camera = lambda: camera.append('close')
        self.s.command('camera_open', {}, 'browser-one')
        self.assertEqual(camera, ['main'])
        self.assertEqual(self.s.state, 'IDLE')
        self.assertFalse(self.s.locked)
        self.assertEqual(self.starts, [])
        self.s.command('camera_close', {}, 'browser-one')
        self.s.command('stop', {}, 'browser-one')
        self.assertEqual(camera, ['main', 'close', 'close'])
        self.s.command('qr_camera_open', {}, 'browser-one')
        self.assertEqual(camera[-1], 'qr')
        self.assertEqual(self.s.state, 'IDLE')
        self.assertIsNone(self.s.qr_result)
        self.begin()
        for action in ('camera_open', 'qr_camera_open', 'camera_close'):
            with self.assertRaisesRegex(ValueError, '仅待命'):
                self.s.command(action, {}, 'browser-one')

    def batch_done(self):
        area, operation, _, colors, positions = batch_spec(WORK_STAGE[self.s.route_index], self.s.code, self.s.first_layer)
        rows = [r[1:] for r in expected_steps(operation, colors, positions)]
        real = self.s.real_actions[area]
        self.status = dict(done=True, label='整批完成', velocity=[0.,0.,0.], confirmed=rows,
            events=['真实动作完成' if real else '停留1秒替代完成'],
            results=[dict(operation=r[0],index=r[1],color=r[2],position=r[3],
                outcome='REAL_COMPLETED' if real else 'DWELL_COMPLETED') for r in rows])
        self.refresh()
        self.s.tick()

    def test_all_switch_combinations_and_both_rounds_follow_formal_code(self):
        for values in itertools.product((True,False), repeat=3):
            self.setUp()
            switches = dict(zip(('raw','coarse','storage'), values))
            self.begin(switches, mode='screen')
            for index, goal in enumerate([1,2,4,6,8,4,6,8,3,3]):
                self.assertEqual(self.s.routes[index]['goal'], goal)
                self.complete()
                if self.s.state == 'SCANNING':
                    self.assertIn('首帧', self.s.reason)
                    self.assertIn('首帧', self.s.current_action)
                    for _ in range(3):
                        self.clock.now += 10
                        self.refresh()
                        self.s.tick()
                        self.assertEqual(self.s.state, 'SCANNING')
                        self.assertEqual(self.io.commands[-1], (0.,0.,0.))
                    self.io.qr_value='156+231+516+312'
                    self.s.tick()
                elif self.s.state == 'WORKING':
                    self.batch_done()
            self.assertEqual(self.s.state, 'FINISHED', self.s.reason)
            self.assertEqual([s[0] for s in self.starts], ['raw','coarse','storage']*2)
            self.assertEqual([s[1] for s in self.starts], ['RAW_PICK','COARSE_PROCESS','STORAGE_PLACE',
                                                          'RAW_PICK','COARSE_PROCESS','STORAGE_STACK'])
            self.assertEqual(self.starts[-1][3], (3,2,1))  # Same-color positions in first layer.
            self.assertEqual(len(self.s.results),24)
            self.assertEqual(self.counts_shown, [(6*switches['raw']+6*switches['coarse'],
                                                 6*switches['coarse']+6*switches['storage'])])
            for i, start in enumerate(self.starts):
                self.assertEqual(start[5], 'real' if switches[start[0]] else 'simulated')
            with self.assertRaisesRegex(ValueError,'先复位'):
                self.s.command('start',dict(mode='web'),'browser-one')
            before=len(self.s.log)
            for i in range(100):self.s._record('完整日志条目'+str(i))
            self.assertEqual(len(self.s.snapshot()['log']),before+100)
            self.assertEqual(len(self.s.snapshot()['events']),80)

    def test_screen_countdown_gate_and_configuration_locked_until_reset(self):
        self.begin(start=3, goal=1, mode='screen')
        self.assertEqual(self.s.config_mode, 'screen')
        snapshot=self.s.snapshot()
        self.s.screen_message('CFG',(1,3))
        with self.assertRaisesRegex(ValueError,'先停止'):
            self.s.command('start',dict(mode='web'),'browser-one')
        self.assertEqual(self.s.snapshot()['settings'],snapshot['settings'])
        self.plan()
        self.s.command('stop',{},'observer')
        self.io.reply('get_runtime',{'runtime':self.rt})
        self.assertEqual(self.s.state,'STOPPED')
        self.s.command('reset',{},'browser-one')
        self.assertFalse(self.s.locked)
        self.assertIsNone(self.s.qr_result)
        self.assertEqual(self.s.state,'IDLE')

    def test_work_stop_requires_region_release_and_never_resumes(self):
        self.begin()
        self.complete(); self.complete()
        self.io.qr_value='156+231+516+312';self.s.tick();self.complete()
        self.assertEqual(self.s.state,'WORKING')
        self.io.work_free=False
        self.s.command('stop',{},'observer')
        self.assertTrue(self.io.work_cancelled)
        self.io.reply('get_runtime',{'runtime':self.rt})
        self.assertEqual(self.s.state,'STOPPING')
        self.io.work_free=True
        self.s.tick();self.io.reply('get_runtime',{'runtime':self.rt})
        self.assertEqual(self.s.state,'STOPPED')
        with self.assertRaises(ValueError):self.s.command('start',{},'browser-one')
        self.assertFalse(any(name=='plan' for name,_,_ in self.io.calls))

    def test_missing_first_layer_visual_never_finishes_stack_or_invents_records(self):
        self.begin(dict.fromkeys(('raw','coarse','storage'),False))
        self.complete();self.complete()
        self.io.qr_value='156+231+516+312';self.s.tick()
        for _ in range(5):self.complete();self.batch_done()
        self.complete()
        self.status=dict(done=False,label='等待第一层真实同色物料',velocity=[0.,0.,0.],confirmed=[],events=[],results=[])
        count=len(self.s.log)
        for _ in range(10):self.refresh();self.s.tick()
        self.assertEqual(self.s.state,'WORKING')
        self.assertEqual(len(self.s.results),21)
        self.assertLessEqual(len(self.s.log)-count,1)
        self.assertIn('等待',self.s.reason)

    def test_invalid_options_rejected(self):
        for raw in (dict(mode='free'),dict(real_actions={}),dict(real_actions=dict(raw=True,coarse=1,storage=True))):
            with self.assertRaises(ValueError):run_options(raw)

    def test_lidar_scan_after_exit_merges_map_and_waits_for_planner_ack(self):
        self.io.use_lidar=True
        started=[]
        self.io.start_lidar=lambda run_id: started.append(run_id)
        self.begin()
        self.complete()
        self.assertEqual(self.s.state,'WORKING')
        self.assertTrue(self.s.radar)
        self.assertEqual(started,[self.s.run_id])
        self.status=dict(done=True,blocked_edges=[(1,2)],label='扫描完成',velocity=[0.,0.,0.])
        self.s.tick()
        self.assertEqual(self.s.state,'WORKING')
        self.assertEqual(self.io.maps[-1],(self.s.run_id,2,[(1,2),(4,7)]))
        self.rt.update(map_revision=2,blocked_edges=[dict(a=1,b=2),dict(a=4,b=7)])
        self.refresh();self.s.tick()
        self.assertEqual(self.s.state,'NAVIGATING')
        self.assertEqual(self.s.route_index,1)


if __name__ == '__main__':unittest.main()
