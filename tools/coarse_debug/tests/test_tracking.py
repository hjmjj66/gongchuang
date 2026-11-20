"""Exercise continuous tracking and the real manual-action handoff without hardware."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from agv_perception.work_config import read_work_config

import unittest

import yaml

from agv_perception.work_control import Observation
from coarse_debug.tracking import TrackingTask, parse_tracking_target


CONFIG = read_work_config((Path(__file__).parents[1]/'config/debug.yaml'))


class TrackingTests(unittest.TestCase):
    def make_task(self, area='coarse', visit=1):
        p = deepcopy(CONFIG[area])
        p['vision']['target_uv_px'] = [100., 100.]
        if area == 'storage':
            p['vision']['stack_target_uv_px'] = [100., 100.]
        kind = 'material' if area == 'raw' or (area == 'storage' and visit == 2) else 'ring'
        return TrackingTask(p, 'tracking-test', 2, kind, 0., area, visit)

    def step(self, task, now, point=(130., 120.), **kwargs):
        obs = Observation(task.kind, int(now*1000), now, (201,201),
                          {2:point} if task.kind == 'ring' and point else {},
                          ((2,point),) if task.kind == 'material' and point else ())
        task.tick(now, observation=obs, **kwargs)
        self.assertEqual(task.velocity, (0.,0.,0.))
        return task.arm_velocity

    def test_all_areas_track_forever_and_use_their_existing_gains_and_limits(self):
        for area, visit in (('raw',1), ('coarse',1), ('storage',1), ('storage',2)):
            with self.subTest(area=area, visit=visit):
                task = self.make_task(area, visit)
                c = task.p['gantry_servo']
                yaw_key, fb_key = ('yaw_pid','fb_pid') if area == 'raw' else ('u_pid','v_pid')
                yaw, fb = self.step(task, .05)
                self.assertAlmostEqual(yaw, -min(30*c[yaw_key]['kp'], c['max_yaw_rad_s']))
                self.assertAlmostEqual(fb, -min(20*c[fb_key]['kp'], c['max_fb_rpm']))
                for i in range(1,30):
                    self.step(task, .05+i*.05, (100.,100.))
                    self.assertIsNone(task.arm_request)
                    self.assertFalse(task.finished)
                self.assertEqual(task.state, 'TRACK_HOLD')
                self.step(task, 1.6, (80.,80.))
                self.assertEqual(task.state, 'TRACKING')
                self.assertGreater(task.arm_velocity[0], 0.)
                self.assertGreater(task.arm_velocity[1], 0.)

    def test_missing_target_waits_without_search_restore_or_other_color(self):
        task = self.make_task('raw')
        self.step(task, .1)
        self.step(task, .2, None)
        self.assertFalse(task.servo_enabled)
        self.assertEqual(task.arm_velocity, (0.,0.))
        for now in (1.,10.,1000.):
            obs = Observation('material',int(now*1000),now,(201,201),{},((1,(80.,80.)),))
            task.tick(now, observation=obs)
            self.assertEqual(task.state, 'TRACK_WAIT')
            self.assertIsNone(task.arm_request)
            self.assertEqual(task.arm_velocity, (0.,0.))
            self.assertTrue(all(pid.integral == 0. for pid in task._pids))
        self.step(task,1000.1,(80.,80.))
        self.assertGreater(task.arm_velocity[0],0.)

    def test_ring2_fb_only_ignores_horizontal_error_and_keeps_following_until_stopped(self):
        for area in ('coarse', 'storage'):
            p = deepcopy(CONFIG[area])
            p['vision']['target_uv_px'] = [100., 100.]
            task = TrackingTask(p, 'fb-only', 2, 'ring', 0., area, fb_only=True)
            self.assertEqual(task.vision_target, 2)
            yaw, fb = self.step(task, .1, (180., 120.))
            self.assertEqual(yaw, 0.)
            self.assertAlmostEqual(fb, -min(20*p['gantry_servo']['v_pid']['kp'],
                                          p['gantry_servo']['max_fb_rpm']))
            self.step(task, .2, (180., 100.))
            self.assertEqual(task.state, 'TRACK_HOLD')
            self.assertFalse(task.finished)
            self.step(task, 10., (20., 80.))
            self.assertEqual(task.state, 'TRACKING')
            self.assertEqual(task.arm_velocity[0], 0.)
            self.assertGreater(task.arm_velocity[1], 0.)
            task.tick(10.1, observation=Observation('ring', 10100, 10.1, (201,201),
                                                  {1:(100.,100.), 3:(100.,100.)}))
            self.assertEqual(task.state, 'TRACK_WAIT')
            self.assertEqual(task.arm_velocity, (0.,0.))
            self.assertFalse(task.servo_enabled)
            self.step(task, 10.2, (180., 120.))
            self.assertLess(task.arm_velocity[1], 0.)
            self.assertIsNone(task.arm_request)
            task.stop(10.3, '网页停止', False)
            self.step(task, 10.4)
            self.assertEqual(task.state, 'STOPPED')
            self.assertEqual(task.arm_velocity, (0.,0.))

    def test_frame_freshness_sequence_center_default_and_stop(self):
        task = self.make_task()
        task.p['vision']['target_uv_px'] = []
        self.step(task,.1,(100.,100.))
        self.assertEqual(task.state,'TRACK_HOLD')
        self.step(task,.1,(20.,20.))  # Duplicate inference cannot update the controller.
        self.assertEqual(task.arm_velocity,(0.,0.))
        obs = Observation('ring',200,.2,(201,201),{2:(20.,20.)})
        with self.assertRaisesRegex(RuntimeError,'过期'):
            task.tick(100.,observation=obs)
        self.assertFalse(task.servo_enabled)
        task.stop(101.,'网页停止',False)
        self.assertEqual(task.state,'STOPPED')
        self.step(task,102.)
        self.assertEqual(task.arm_velocity,(0.,0.))

    def test_manual_programs_wait_for_new_idle_and_complete_once_without_resuming(self):
        for area,visit,command in (('raw',1,4),('coarse',1,6),('storage',1,6),('storage',2,7)):
            with self.subTest(area=area,visit=visit):
                task = self.make_task(area,visit)
                self.step(task,.1)
                task.request_action(.2)
                self.assertFalse(task.servo_enabled)
                self.assertEqual(task.arm_velocity,(0.,0.))
                arm = SimpleNamespace(idle=True,pending=False,command_id='',state='IDLE',reason='')
                task.tick(.4,arm=arm,arm_at=.1)
                self.assertIsNone(task.arm_request)  # Old idle sample is insufficient.
                arm.idle=False
                task.tick(.5,arm=arm,arm_at=.5)
                self.assertIsNone(task.arm_request)
                arm.idle=True
                task.tick(.6,arm=arm,arm_at=.6)
                self.assertEqual(task.arm_request['cmd'],command)
                self.assertEqual(task.state,'TRACK_ACTION')
                with self.assertRaises(ValueError):
                    task.request_action(.7)
                arm.state='COMPLETED'
                task.tick(.8,arm=arm)
                self.assertFalse(task.finished)  # A different command cannot complete this task.
                arm.command_id=task.arm_request['command_id']
                task.tick(.9,arm=arm)
                self.assertTrue(task.finished)
                self.assertEqual(len(task.confirmed),1)
                self.step(task,1.)
                self.assertFalse(task.servo_enabled)
                self.assertEqual(task.arm_velocity,(0.,0.))

    def test_stop_handoff_timeout_and_rejected_action_fail(self):
        task=self.make_task()
        task.request_action(.1)
        with self.assertRaisesRegex(RuntimeError,'停稳'):
            task.tick(100.)
        task=self.make_task()
        task.request_action(.1)
        arm=SimpleNamespace(idle=True,pending=False,command_id='',state='IDLE',reason='动作被拒绝')
        task.tick(.3,arm=arm,arm_at=.3)
        arm.command_id,arm.state=task.arm_request['command_id'],'FAILED'
        with self.assertRaisesRegex(RuntimeError,'被拒绝'):
            task.tick(.4,arm=arm)

    def test_target_validation(self):
        self.assertEqual(parse_tracking_target({'target':6},'material'),6)
        for kind,target in (('ring',4),('material',7),('ring',True),('ring','2'),('ring',None)):
            with self.assertRaises(ValueError):
                parse_tracking_target({'target':target},kind)

    def test_raw_prediction_updates_between_frames_and_stops_on_loss_and_expiry(self):
        task = self.make_task('raw')
        self.assertTrue(task.p['motion_prediction']['enabled'])
        for i in range(12):
            self.step(task,1.+i*.04,(110.+i,100.))
        now = 1.+11*.04
        obs = Observation('material',int(now*1000),now,(201,201),{},((2,(121.,100.)),))
        state = task._raw_servo.filter.statePost.copy()
        previous = task.arm_velocity[0]
        task.tick(now+.02,observation=obs)
        self.assertLess(task.arm_velocity[0],previous)
        self.assertEqual(task._raw_servo.filter.statePost.tolist(),state.tolist())
        self.assertIsNone(task.arm_request)
        task.tick(now+.16,observation=obs)
        self.assertEqual(task.arm_velocity,(0.,0.))
        self.assertEqual(task.state,'TRACK_WAIT')
        self.assertFalse(task.servo_enabled)
        self.step(task,now+.2,(100.,100.))
        self.assertEqual(task.diagnostic['pixel_velocity_px_s'],(0.,0.))
        self.step(task,now+.24,None)
        self.assertIsNone(task._raw_servo.filter)
        self.assertEqual(task.arm_velocity,(0.,0.))
        self.step(task,now+.28,(100.,100.))
        self.assertEqual(task.state,'TRACK_HOLD')
        self.assertEqual(task.diagnostic['pixel_velocity_px_s'],(0.,0.))


if __name__ == '__main__':
    unittest.main()
