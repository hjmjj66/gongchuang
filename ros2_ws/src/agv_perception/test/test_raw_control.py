"""Exercise fixed-base raw picking, base approach and returns and actual visual control."""
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from agv_perception.raw_control import RawTask, read_raw_config, validate_raw_config
from agv_perception.work_config import read_work_config
from agv_perception.work_control import GantryPose, Observation, Pose


def calibrated_raw_config():
    p = read_work_config(Path(__file__).resolve().parents[4]/'tools/coarse_debug/config/debug.yaml')['raw']
    p['vision']['target_uv_px'] = [100., 100.]
    p['motion_prediction']['enabled'] = False
    p['gantry_servo'].update(max_yaw_rad_s=.5,max_fb_rpm=20.,yaw_pid=dict(kp=.02,ki=0.,kd=0.),fb_pid=dict(kp=.5,ki=0.,kd=0.))
    p['grasp_gate'].update(qualified_frames=4,tolerance_px=2.,small_yaw_rad_s=.02,small_fb_rpm=.5)
    return p


class RawTests(unittest.TestCase):
    def setUp(self):
        self.p = calibrated_raw_config()
        self.now, self.sequence = 0., 0
        self.xy = (1., 2.)
        self.actual = (120, .3)
        self.task = RawTask(self.p, 'raw-test', [1, 3, 4], self.now)
        self.assertEqual(self.task.state, 'ENTER_WORK_POSE')
        self.assertEqual(self.task.arm_request['cmd'], 3)
        self.step(arm=True)

    def step(self, points=None, arm=False, dt=.05):
        self.now += dt
        self.sequence += 1
        self.obs = None if points is None else Observation('material', self.sequence, self.now,
                                                           (201, 201), {}, tuple(points.items()))
        status = (SimpleNamespace(command_id=self.task.arm_request['command_id'],state='COMPLETED') if arm else
                  SimpleNamespace(command_id='',state='IDLE',idle=True,pending=False))
        velocity = self.task.tick(self.now, Pose(*self.xy,0.,self.now),self.obs,status,
                                 GantryPose(*self.actual,self.now))
        if self.task.state not in ('APPROACH', 'RETURN_REFERENCE'):
            self.assertEqual(velocity, (0.,0.,0.))
        if arm and self.task.state == 'APPROACH':
            self.step()
            self.step(dt=.11)
            self.xy = self.task.approach_target
            self.step()
            self.step(dt=.11)
        return velocity

    def wait_required(self):
        self.assertEqual(self.task.state, 'WAIT_REQUIRED')

    def pick(self, color):
        self.step({})
        for _ in range(4):
            self.step({color:(100.,100.)})
        self.assertEqual(self.task.state,'PICK')
        self.assertEqual(self.task.arm_request['cmd'],4)
        self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_unrequested_colors_never_start_initial_follow(self):
        for color in (0, 2, 3, 4, 5, 6):
            self.step({color: (130., 80.)})
            self.assertEqual(self.task.state, 'WAIT_REQUIRED')
            self.assertEqual(self.task.arm_velocity, (0., 0.))
            self.assertIsNone(self.task.arm_request)
        self.step({1: (130., 80.)})
        self.assertEqual(self.task.state, 'TRACK_REQUIRED')

    def test_empty_images_wait_forever_without_base_scan_or_work_recovery(self):
        for _ in range(5):
            self.step({},dt=1000.)
        self.assertEqual(self.task.state,'WAIT_REQUIRED')
        self.assertIsNone(self.task.arm_request)
        self.assertIsNotNone(self.task.reference)




    def test_four_distinct_frames_normal_output_gate_without_m0(self):
        self.step({1:(120.,120.)})
        self.assertLess(self.task.arm_velocity[0],0.)
        self.assertLess(self.task.arm_velocity[1],0.)
        self.assertEqual(self.task.count,0)
        self.step({})
        for _ in range(3):self.step({1:(100.,100.)})
        self.task.tick(self.now+.01,Pose(*self.xy,0.,self.now),self.obs)
        self.assertEqual(self.task.count,3)
        self.actual=(432,-.7)
        self.step({1:(100.,100.)})
        self.assertFalse(hasattr(self.task, 'm0'))
        self.assertEqual(self.task.state,'PICK')
        self.step(arm=True)
        self.actual=(999,1.2)
        self.pick(3)
        self.assertFalse(hasattr(self.task, 'm0'))

    def test_configured_frame_count_uses_only_distinct_qualified_frames(self):
        for required in (1, 2, 6):
            with self.subTest(required=required):
                self.p['grasp_gate']['qualified_frames'] = required
                validate_raw_config(self.p)
                self.task = RawTask(self.p, 'raw-frames', [1, 3, 4], self.now)
                self.step(arm=True)
                for count in range(1, required):
                    self.step({1: (100., 100.)})
                    self.assertEqual(self.task.count, count)
                    self.task.tick(self.now+.01, Pose(*self.xy, 0., self.now), self.obs)
                    self.assertEqual(self.task.count, count)
                    self.assertIsNone(self.task.arm_request)
                    self.assertIn(f'等待{required}帧', self.task.diagnostic['wait_reason'])
                if required > 1:
                    self.step({})
                    self.assertEqual(self.task.count, 0)
                    for _ in range(required-1):
                        self.step({1: (100., 100.)})
                self.step({1: (100., 100.)})
                self.assertEqual(self.task.state, 'PICK')
                self.assertEqual(self.task.arm_velocity, (0., 0.))

    def test_loss_before_first_pick_waits_without_m0_and_one_frame_resumes(self):
        self.step({1:(120.,100.)})
        self.step({})
        for _ in range(4):self.step({3:(120.,100.)},dt=1000.)
        self.assertEqual(self.task.state,'WAIT_REQUIRED')
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertIsNone(self.task.arm_request)
        self.step({1:(120.,100.)})
        self.assertEqual(self.task.state,'TRACK_REQUIRED')
        self.assertTrue(any(self.task.arm_velocity))



    def test_tracking_loss_waits_in_place_clears_gate_and_resumes_on_one_frame(self):
        self.pick(1)
        self.step(arm=True)
        self.p['motion_prediction']['enabled'] = True
        for _ in range(3):
            self.step({3:(100.,100.)})
        self.assertEqual(self.task.count,3)
        self.actual=(0,0.)
        self.step({4:(100.,100.)})
        self.assertEqual(self.task.state,'WAIT_REQUIRED')
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertEqual(self.task.count,0)
        self.assertIsNone(self.task._gantry.filter)
        self.task.tick(self.now+.01,Pose(*self.xy,0.,self.now),self.obs,
                       gantry=GantryPose(*self.actual,self.now+.01))
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.step({3:(100.,100.)})
        self.assertEqual(self.task.state,'TRACK_REQUIRED')
        self.assertEqual(self.task.count,1)
        self.assertIsNone(self.task.arm_request)


    def test_completion_only_after_stow_and_base_return(self):
        for color in (1,3,4):
            self.pick(color)
            wrong=SimpleNamespace(command_id='old',state='COMPLETED')
            self.task.tick(self.now,Pose(*self.xy,0.,self.now),arm=wrong)
            self.assertEqual(self.task.state,'PICK')
            self.step(arm=True)
        self.assertEqual(self.task.state,'STOW_ARM')
        self.assertEqual(self.task.arm_request['cmd'],2)
        self.assertFalse(self.task.finished)
        self.step(arm=True)
        self.assertEqual(self.task.state,'RETURN_REFERENCE')
        self.xy = self.task.reference
        self.step()
        self.step(dt=.11)
        self.assertEqual(self.task.state,'DONE')
        self.assertTrue(self.task.returned)
        self.assertFalse(self.task.reference_return_timed_out)
        self.assertIsNotNone(self.task.reference)
        second=RawTask(self.p,'second',[4,1,3],self.now)
        self.assertFalse(hasattr(second, 'm0'))
        self.assertEqual(second.state,'ENTER_WORK_POSE')

    def test_prediction_between_frames_never_corrects_again_or_adds_grasp_frames(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        for i in range(8):
            self.step({1:(110.+i,100.)})
        state = self.task._gantry.filter.statePost.copy()
        command = self.task.arm_velocity
        self.task.tick(self.now+.02,Pose(*self.xy,0.,self.now),self.obs)
        self.assertLess(self.task.arm_velocity[0],command[0])
        self.assertEqual(self.task._gantry.filter.statePost.tolist(),state.tolist())
        self.assertEqual(self.task.count,0)
        self.step({})
        for _ in range(3):
            self.step({1:(100.,100.)})
        self.assertEqual(self.task.count,3)
        for dt in (.005,.01,.02):
            self.task.tick(self.now+dt,Pose(*self.xy,0.,self.now),self.obs)
        self.assertEqual(self.task.count,3)
        self.assertIsNone(self.task.arm_request)
        self.step({1:(100.,100.)})
        self.assertEqual(self.task.state,'PICK')
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertIsNone(self.task._gantry.filter)

    def test_prediction_gate_uses_raw_center_and_expires_before_image_fault(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        for _ in range(3):
            self.step({1:(100.,100.)})
        self.step({1:(102.1,100.)})
        self.assertLess(self.task.diagnostic['filtered_error_uv'][0],2.)
        self.assertEqual(self.task.count,0)
        self.task.tick(self.now+.16,Pose(*self.xy,0.,self.now),self.obs)
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.assertFalse(self.task.diagnostic['prediction_active'])
        self.assertIsNone(self.task.arm_request)
        self.step({1:(100.,100.)},dt=.2)
        self.assertEqual(self.task.count,1)
        self.assertEqual(self.task.diagnostic['pixel_velocity_px_s'],(0.,0.))

    def test_prediction_loss_pose_and_color_changes_clear_velocity(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        for i in range(8):
            self.step({1:(120.+i,110.)})
        self.step({})
        self.assertIsNone(self.task._gantry.filter)
        self.task.tick(self.now+.01,Pose(*self.xy,0.,self.now),self.obs)
        self.assertFalse(self.task.servo_enabled)
        self.step({1:(100.,100.)})
        self.assertEqual(self.task.diagnostic['pixel_velocity_px_s'],(0.,0.))
        for _ in range(3):
            self.step({1:(100.,100.)})
        self.step(arm=True)
        self.assertEqual(self.task.vision_target,3)
        self.step({3:(100.,100.)})
        self.assertEqual(self.task.count,1)
        self.assertEqual(self.task.diagnostic['pixel_velocity_px_s'],(0.,0.))
        self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_prediction_gap_without_an_intermediate_tick_restarts_gate(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        for _ in range(3):
            self.step({1:(100.,100.)})
        self.step({1:(100.,100.)},dt=.2)
        self.assertEqual(self.task.state,'TRACK_REQUIRED')
        self.assertEqual(self.task.count,1)

    def test_prediction_rejects_bad_centers_and_nonincreasing_frame_time_at_entry(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        self.step({1:(120.,100.)})
        for point in ((float('nan'),100.),(100.,float('inf')),(100.,)):
            with self.subTest(point=point), self.assertRaisesRegex(ValueError,'中心坐标'):
                self.step({1:point})
        obs = Observation('material',self.sequence+1,self.now-.01,(201,201),{},((1,(100.,100.)),))
        with self.assertRaisesRegex(ValueError,'时间未递增'):
            self.task.tick(self.now,Pose(*self.xy,0.,self.now),obs,gantry=GantryPose(120,.3,self.now))

    def test_prediction_closed_loop_with_moving_camera_delayed_frames_and_target_stop(self):
        self.p['motion_prediction']['enabled'] = True
        self.wait_required()
        start = self.now
        camera = [0.,0.]
        pending, obs = [], None
        for i in range(1,501):
            elapsed = i*.02
            self.now = start+elapsed
            yaw, fb = self.task.arm_velocity
            camera[0] -= yaw*100.*.02
            camera[1] -= fb*5.*.02
            if i % 2 == 0:
                self.sequence += 1
                target = (15.+15.*min(elapsed,1.5),8.)
                point = tuple(100.+p-c for p,c in zip(target,camera))
                pending.append(Observation('material',self.sequence,self.now,(201,201),{},((1,point),)))
            while pending and self.now-pending[0].captured_at >= .06:
                obs = pending.pop(0)
            self.task.tick(self.now,Pose(*self.xy,0.,self.now),obs,gantry=GantryPose(120,.3,self.now))
            if self.task.state == 'PICK':
                break
        self.assertEqual(self.task.state,'PICK')
        self.assertGreater(elapsed,1.5)
        self.assertLess(math.hypot(37.5-camera[0],8.-camera[1]),self.p['grasp_gate']['tolerance_px'])
        self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_stale_frame_and_cancel_stop_both_outputs(self):
        self.step({1:(120.,100.)})
        for delay in (.6, 3600.):
            self.task.tick(self.now+delay,Pose(0.,0.,0.,self.now+delay),self.obs)
            self.assertFalse(self.task.finished)
            self.assertEqual(self.task.arm_velocity,(0.,0.))
            self.assertFalse(self.task.servo_enabled)
        self.task.stop(self.now,'cancel',failed=False)
        self.step({1:(100.,100.)})
        self.assertEqual(self.task.state,'STOPPED')
        self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_config_requires_positive_frame_count_and_resolvable_base_distance(self):
        validate_raw_config(self.p)
        formal=read_raw_config(Path(__file__).parents[1]/'config/raw.yaml')
        validate_raw_config(formal)
        formal['base_motion']['distance_mm']=None
        with self.assertRaisesRegex(ValueError,'未标定'):validate_raw_config(formal)
        self.p['grasp_gate']['qualified_frames']=3
        validate_raw_config(self.p)
        for value in (0, -1, 1.5, 4., True, '4'):
            self.p['grasp_gate']['qualified_frames'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_raw_config(self.p)
        self.p['grasp_gate']['qualified_frames']=4
        self.p['base_motion']['position_tolerance_m']=.012
        with self.assertRaisesRegex(ValueError,'小于前进距离'):validate_raw_config(self.p)


if __name__=='__main__':
    unittest.main()


class BaseMotionTests(unittest.TestCase):
    def run_base(self, task, xy, yaw, now=0.):
        states = []
        for _ in range(1000):
            now += .02
            vx, vy, _ = task.velocity
            cs, sn = math.cos(yaw), math.sin(yaw)
            xy = (xy[0]+(cs*vx-sn*vy)*.02, xy[1]+(sn*vx+cs*vy)*.02)
            task.tick(now, Pose(*xy, yaw, now, vx, vy))
            states.append(task.state)
            if task.state == 'WAIT_REQUIRED' or task.finished:
                break
        return xy, now, states

    def test_approach_and_return_at_multiple_headings_and_distances(self):
        for yaw in (0., math.pi/2, -.8):
            for distance in (12., 25.):
                p = calibrated_raw_config()
                p['base_motion']['distance_mm'] = distance
                task = RawTask(p, 'base', (1, 2, 3), 0.)
                task._wait_arm(.01, SimpleNamespace(command_id=task.arm_request['command_id'], state='COMPLETED'))
                xy, now, _ = self.run_base(task, (1., 2.), yaw)
                self.assertEqual(task.state, 'WAIT_REQUIRED')
                self.assertAlmostEqual(xy[0], 1.+distance/1000.*math.cos(yaw), delta=.001)
                self.assertAlmostEqual(xy[1], 2.+distance/1000.*math.sin(yaw), delta=.001)
                task._enter('STOW_ARM', now)
                task._wait_arm(now+.01, SimpleNamespace(command_id=task.arm_request['command_id'], state='COMPLETED'))
                xy, now, _ = self.run_base(task, xy, yaw, now+.01)
                self.assertEqual(task.state, 'DONE')
                self.assertTrue(task.returned)
                self.assertAlmostEqual(xy[0], 1., delta=.001)
                self.assertAlmostEqual(xy[1], 2., delta=.001)

    def test_deferred_exit_stops_and_preserves_actual_work_offset(self):
        p = calibrated_raw_config()
        task = RawTask(p, 'deferred', (1, 2, 3), 0., defer_reference_return=True)
        task._wait_arm(.01, SimpleNamespace(command_id=task.arm_request['command_id'], state='COMPLETED'))
        xy, now, _ = self.run_base(task, (0., 0.), 0.)
        work = xy
        task._enter('STOW_ARM', now)
        task._wait_arm(now+.01, SimpleNamespace(command_id=task.arm_request['command_id'], state='COMPLETED'))
        xy, now, _ = self.run_base(task, xy, 0., now+.01)
        self.assertTrue(task.reference_return_deferred)
        self.assertFalse(task.returned)
        self.assertEqual(xy, work)
        self.assertEqual(task.velocity, (0., 0., 0.))

    def test_missing_feedback_and_motion_timeout_do_not_start_picking(self):
        p = calibrated_raw_config()
        task = RawTask(p, 'failure', (1, 2, 3), 0.)
        task._enter('APPROACH', 0.)
        with self.assertRaisesRegex(RuntimeError, '里程反馈'):
            task.tick(.1, None)
        with self.assertRaisesRegex(RuntimeError, '超时'):
            task.tick(16., Pose(0., 0., 0., 16.))
        self.assertEqual(task.confirmed, [])
