"""Exercise real coarse controllers with fresh image and actual gantry samples."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from agv_perception.coarse_control import (
    CoarseTask, GantryPose, Observation, PID, Pose, line_slope,
    read_coarse_config, validate_coarse_config,
)


def calibrated_config():
    p = read_coarse_config(Path(__file__).parents[1]/'config/coarse.yaml')
    p.pop('gantry_presets', None)  # Exercise visual alignment independently of saved field positions.
    p['vision']['target_uv_px'] = [100., 100.]
    p['base_servo'].update(tolerance_px=2., settle_frames=2)
    p['gantry_servo']['tolerance_px'] = 2.
    for section in ('base_servo', 'gantry_servo'):
        p[section]['timeout_s'] = 5.
    p['angle_alignment'].update(slope_tolerance=.02, settle_frames=2)
    return p


class CoarseTests(unittest.TestCase):
    def setUp(self):
        self.p = calibrated_config()
        self.task = CoarseTask(self.p, 'coarse-test', [5, 1, 6], [2, 3, 1], 0.)
        self.now, self.sequence = 0., 0
        self.xy, self.rt = (0., 0.), (0., 0.)
        self.yaw, self.vx = 0., 0.
        self.actions = []

    def step(self, rings=None, arm=False, dt=.05, idle=True):
        self.now += dt
        self.sequence += 1
        obs = None if rings is None else Observation('ring', self.sequence, self.now, (201,201), rings)
        status = SimpleNamespace(command_id='', state='IDLE', idle=idle, pending=False)
        if arm:
            self.actions.append(self.task.arm_request['cmd'])
            status.command_id = self.task.arm_request['command_id']
            status.state = 'COMPLETED'
        return self.task.tick(self.now, Pose(*self.xy, self.yaw, self.now, self.vx), obs, status,
                              GantryPose(*self.rt, self.now))

    def align(self):
        rings = {1:(20.,100.), 2:(100.,100.), 3:(180.,100.)}
        state = self.task.state
        for _ in range(10):
            self.step(rings)
            if self.task.state != state:
                return
        self.fail(f'Alignment did not complete: {state}')

    def enter_work(self):
        self.step(arm=True)
        self.align()
        self.assertEqual(self.task.state, 'CAPTURE_REFERENCE')
        for _ in range(4): self.step()
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.align()
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.assertEqual(self.task.reference, self.xy)
        self.assertIsNone(self.task.arm_request)

    def align_and_place(self, ring, rt):
        self.rt = rt
        for _ in range(4):
            self.step({ring:(100.,100.)})
        self.assertEqual(self.task.state, 'STOP_PLACE')
        self.assertNotIn(ring, self.task.placements)
        self.step(dt=.15)
        self.assertEqual(self.task.state, 'PLACE')
        self.assertEqual(self.task.placements[ring], rt)
        self.rt = (0., 0.)  # Fixed program returns to high work pose before completion.
        self.step(arm=True)
        self.assertEqual(self.task.placements[ring], rt)

    def test_full_batch_records_before_place_and_retrieves_same_order_without_vision(self):
        self.xy = (.04, -.03)
        self.enter_work()
        points = {2:(12., .15), 3:(25., -.2), 1:(5., .3)}
        for i, ring in enumerate((2,3,1)):
            self.assertEqual(self.task.state, 'ALIGN_RING')
            self.assertIsNone(self.task.arm_request)
            self.align_and_place(ring, points[ring])
        self.assertEqual(self.task.placements, points)
        for ring in (2,3,1):
            self.assertEqual(self.task.state, 'RETURN_ITEM')
            self.step()  # No image or material sample during feedback return.
            self.assertTrue(self.task.servo_enabled)
            self.assertEqual(self.task.velocity, (0.,0.,0.))
            self.rt = points[ring]
            self.step()
            self.assertEqual(self.task.state, 'STOP_PICK')
            self.step(dt=.15)
            self.assertEqual(self.task.state, 'PICK')
            self.rt = (0.,0.)
            self.step(arm=True)
        self.assertEqual(self.task.state, 'WORK_EXIT')
        self.step(arm=True)
        self.align()
        self.assertEqual(self.task.state, 'STOW')
        self.step(arm=True)
        self.assertEqual(self.task.state, 'RETURN_REFERENCE')
        reference = self.task.reference
        self.xy = (reference[0]+.08, reference[1]-.04)
        self.assertLess(self.step()[0], 0.)
        self.assertGreater(self.task.velocity[1], 0.)
        self.xy = reference
        self.step()
        self.step(dt=.15)
        self.assertEqual(self.task.state, 'DONE')
        self.assertTrue(self.task.returned)
        self.assertEqual(self.task.final_alignment, 'ALIGNED')
        self.assertEqual(self.actions, [3,6,6,6,5,5,5,3,2])
        self.assertEqual([(op,pos) for op,_,_,pos in self.task.confirmed],
                         [('PLACE',2),('PLACE',3),('PLACE',1),('PICK',2),('PICK',3),('PICK',1)])
        self.assertEqual(self.task.reference, reference)

    def test_p0_is_captured_after_angle_and_before_ring2_translation(self):
        self.xy = (1., 2.)
        self.step(arm=True)
        self.align()
        self.assertEqual(self.task.state, 'CAPTURE_REFERENCE')
        self.assertIsNone(self.task.reference)
        self.task.tick(self.now+.01, Pose(99.,99.,0.,self.now-.01))
        self.assertIsNone(self.task.reference)
        self.vx = .02
        self.step(dt=.2)
        self.assertIsNone(self.task.reference)
        self.vx = 0.
        for _ in range(4): self.step()
        self.assertEqual(self.task.reference, (1.,2.))
        self.xy = (1.1, 1.95)
        self.align()
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.assertEqual(self.task.reference, (1.,2.))

    def test_deferred_exit_stows_then_confirms_fresh_stationary_feedback_without_return(self):
        self.enter_work()
        self.task.defer_reference_return = True
        self.task._enter('STOW', self.now)
        self.xy = (.2, -.3)
        self.assertEqual(self.step(), (0., 0., 0.))
        self.assertEqual(self.task.state, 'STOW')
        self.step(arm=True)
        self.assertEqual(self.task.state, 'STOP_EXIT')
        self.assertFalse(self.task.reference_return_deferred)
        self.task.tick(self.now+.01, Pose(*self.xy, 0., self.now-.01))
        self.assertIsNone(self.task.zero_since)
        self.vx = .02
        for _ in range(4):
            self.assertEqual(self.step(), (0., 0., 0.))
        self.assertFalse(self.task.finished)
        self.vx = 0.
        self.step()
        self.assertFalse(self.task.finished)
        self.step(dt=.15)
        self.assertEqual(self.task.state, 'DONE')
        self.assertTrue(self.task.reference_return_deferred)
        self.assertFalse(self.task.returned)
        self.assertNotEqual(self.xy, self.task.reference)

    def test_deferred_exit_timeout_or_cancel_does_not_report_ready(self):
        for cancel in (False, True):
            task = CoarseTask(self.p, 'exit-failure', [1,2,3], [1,2,3], 0.,
                              defer_reference_return=True)
            task._enter('STOP_EXIT', 0.)
            if cancel:
                task.stop(.1, 'cancel', failed=False)
                self.assertEqual(task.state, 'STOPPED')
            else:
                with self.assertRaisesRegex(RuntimeError, '停车交接超时'):
                    task.tick(self.p['exit_motion']['timeout_s']+1., Pose(.1,.2,0.,1.))
            self.assertFalse(task.reference_return_deferred)
            self.assertFalse(task.returned)

    def test_return_waits_for_stow_rotates_xy_error_and_confirms_stop(self):
        self.enter_work()
        reference = self.task.reference
        self.task._enter('STOW', self.now)
        self.xy, self.yaw = (.1, -.2), math.pi/2
        self.assertEqual(self.step(), (0.,0.,0.))
        self.step(arm=True)
        self.assertEqual(self.task.state, 'RETURN_REFERENCE')
        self.assertFalse(self.task.returned)
        self.task.tick(self.now+.01, Pose(99.,99.,0.,self.now-.01))
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        vx, vy, wz = self.step()
        self.assertGreater(vx, 0.)
        self.assertGreater(vy, 0.)
        self.assertEqual(wz, 0.)
        self.step(dt=1.)
        self.assertEqual(self.task.state, 'RETURN_REFERENCE')
        self.xy = (.07945, 0.)  # A former fixed-offset endpoint must not finish the return.
        self.step()
        self.assertFalse(self.task.finished)
        self.xy, self.vx = reference, .02
        self.assertEqual(self.step(), (0.,0.,0.))
        self.step(dt=.2)
        self.assertFalse(self.task.finished)
        self.vx = 0.
        self.step()
        self.assertEqual(self.task.state, 'DONE')
        self.assertTrue(self.task.returned)

    def test_return_timeout_cancel_and_already_at_p0(self):
        self.enter_work()
        self.task._enter('STOW', self.now)
        self.step(arm=True)
        self.xy = (.1, .1)
        self.step()
        self.step(dt=16.)
        self.assertEqual(self.task.state, 'STOP_EXIT')
        self.assertTrue(self.task.reference_return_timed_out)
        self.assertFalse(self.task.returned)
        self.step()
        self.step(dt=.2)
        self.assertEqual(self.task.state, 'DONE')
        self.assertFalse(self.task.reference_return_deferred)
        self.task.stop(self.now, 'cancel', False)
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        task = CoarseTask(self.p, 'same-position', [1,2,3], [1,2,3], 0.)
        task.reference = (0.,0.)
        task._enter('STOW', 0.)
        task._arm_completed(.1)
        self.assertEqual(task.state, 'RETURN_REFERENCE')
        task.tick(.2, Pose(0.,0.,0.,.2))
        self.assertFalse(task.finished)
        task.tick(.4, Pose(0.,0.,0.,.4))
        self.assertTrue(task.returned)
        self.assertEqual(task.state, 'DONE')

    def test_slope_and_positive_turn_when_right_is_high(self):
        self.assertAlmostEqual(line_slope([(0,100),(100,90),(200,80)],20), -.1)
        self.assertIsNone(line_slope([(0,1)],20))
        self.step(arm=True)
        velocity = self.step({1:(0.,100.),3:(200.,80.)})
        self.assertGreater(velocity[2],0.)
        self.assertEqual(velocity[:2], (0.,0.))

    def test_both_three_second_timeouts_include_missing_targets(self):
        self.step(arm=True)
        self.step({1:(0.,100.),3:(200.,80.)})
        self.step({}, dt=3.01)
        self.assertEqual(self.task.state, 'CAPTURE_REFERENCE')
        self.assertEqual(self.task.entry_alignment, 'TIMEOUT_SKIPPED')
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        self.task._enter('ALIGN_EXIT', self.now)
        self.step({}, dt=3.01)
        self.assertEqual(self.task.state, 'STOW')
        self.assertFalse(self.task.returned)
        self.assertEqual(self.task.final_alignment, 'TIMEOUT_SKIPPED')

    def test_base_calibration_then_gantry_pixel_signs(self):
        self.task._enter('CENTER_RING2', self.now)
        vx,vy,wz = self.step({2:(200.,0.)})
        self.assertLess(vx,0.)
        self.assertLess(vy,0.)
        self.assertEqual(wz,0.)
        self.task._enter('ALIGN_RING', self.now)
        self.step({2:(200.,0.)})
        self.assertLess(self.task.arm_velocity[0],0.)
        self.assertGreater(self.task.arm_velocity[1],0.)
        self.assertEqual(self.task.velocity,(0.,0.,0.))

    def test_four_distinct_frames_require_small_outputs_and_clear_on_loss(self):
        self.p['gantry_servo']['small_yaw_rad_s'] = .00001
        self.task._enter('ALIGN_RING', self.now)
        self.step({2:(101.,100.)})  # Pixel tolerance alone is insufficient.
        self.assertEqual(self.task.count,0)
        self.step({2:(100.,100.)})
        obs = Observation('ring',self.sequence,self.now,(201,201),{2:(100.,100.)})
        self.task.tick(self.now+.01, Pose(0.,0.,0.,self.now),obs)
        self.assertEqual(self.task.count,1)
        self.step({})
        self.assertEqual(self.task.count,0)
        for _ in range(3):
            self.step({2:(100.,100.)})
        self.assertEqual(self.task.state,'ALIGN_RING')
        self.step({2:(100.,100.)})
        self.assertEqual(self.task.state,'STOP_PLACE')
        self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_missing_target_waits_and_search_is_bounded_in_correct_direction(self):
        for ring, sign in ((1,1),(3,-1),(2,0)):
            self.task.positions = (ring,2,1)
            self.task._enter('ALIGN_RING',self.now)
            self.step({})
            self.assertEqual(math.copysign(1,self.task.arm_velocity[0]) if sign else 0, sign)
            self.step({},dt=1.)
            self.assertEqual(self.task.arm_velocity,(0.,0.))
            self.step({},dt=35.)
            self.assertEqual(self.task.state,'ALIGN_RING')
            self.assertEqual(self.task.arm_velocity,(0.,0.))

    def test_limited_angle_return_stop_recheck_and_total_timeout(self):
        self.task.placements[2] = (10, .1)
        self.task.return_started = self.now
        self.task._enter('RETURN_ITEM',self.now)
        self.rt = (0.,-.1)
        self.step()
        self.assertGreater(self.task.arm_velocity[0],0.)
        self.assertGreater(self.task.arm_velocity[1],0.)
        self.rt = self.task.placements[2]
        self.step()
        self.assertEqual(self.task.state,'STOP_PICK')
        self.rt = (20.,1.)
        self.step(dt=.15)
        self.assertEqual(self.task.state,'RETURN_ITEM')
        self.assertEqual(self.task.return_started,0.)
        self.step(dt=31.)
        self.assertEqual(self.task.state, 'STOP_PICK')
        self.assertTrue(self.task.return_timed_out)
        self.step(dt=.2)
        self.assertEqual(self.task.state, 'PICK')

    def test_ring3_retrieval_timeout_continues_with_screenshot_error(self):
        self.task.item = 1  # The fixture order is 231.
        self.task.placements[3] = (2304, -.39514994621276855)
        self.task.return_started = self.now
        self.task._enter('RETURN_ITEM', self.now)
        self.rt = (2316, -.39543747901916504)
        self.step()
        self.assertEqual(self.task.state, 'RETURN_ITEM')
        self.step(dt=self.p['gantry_motion']['timeout_s'])
        self.assertEqual(self.task.state, 'STOP_PICK')
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.step(dt=.2, idle=False)
        self.assertEqual(self.task.state, 'STOP_PICK')
        self.step()
        self.assertEqual(self.task.state, 'PICK')
        self.step(arm=True)
        self.assertEqual(self.task.state, 'RETURN_ITEM')
        self.assertFalse(self.task.return_timed_out)

    def test_record_requires_fresh_post_stop_feedback_and_idle(self):
        self.enter_work()
        for _ in range(4):
            self.step({2:(100.,100.)})
        self.step(dt=.2,idle=False)
        self.assertEqual(self.task.placements,{})
        arm = SimpleNamespace(idle=True,pending=False)
        self.task.tick(self.now,Pose(0.,0.,0.,self.now),arm=arm,
                       gantry=GantryPose(20.,.2,self.task.entered_at))
        self.assertEqual(self.task.placements,{})
        self.rt = (20.,.2)
        self.step()
        self.assertEqual(self.task.placements[2],self.rt)

    def test_gantry_status_uses_fb_valid_bit_without_requiring_calibration_or_ud(self):
        for flags in range(8):
            board = SimpleNamespace(fb_pulses=-3802, yaw_rad=-.25, pos_flags=flags)
            gantry = GantryPose.from_status(board, 1.)
            if flags & 1:
                self.assertEqual(gantry, GantryPose(-3802, -.25, 1.))
            else:
                self.assertIsNone(gantry)

    def test_invalid_position_cannot_record_return_or_trigger_pick(self):
        self.task.placements[2] = (-3802, -.25)
        self.task.return_started = 0.
        for state in ('STOP_PLACE', 'RETURN_ITEM', 'STOP_PICK'):
            with self.subTest(state=state):
                self.task._enter(state, 0.)
                board = SimpleNamespace(fb_pulses=0, yaw_rad=0., pos_flags=6)
                with self.assertRaisesRegex(RuntimeError, '位置反馈无效'):
                    self.task.tick(.1, Pose(0., 0., 0., .1),
                                   gantry=GantryPose.from_status(board, .1))
                self.assertEqual(self.task.placements[2], (-3802, -.25))
                self.assertIsNone(self.task.arm_request)

    def test_return_uses_signed_pulse_error_and_rpm_limit(self):
        self.task.placements[2] = (-3802, -.25)
        self.task.return_started = 0.
        self.task._enter('RETURN_ITEM', 0.)
        self.rt = (-3000, -.25)
        self.step()
        self.assertEqual(self.task.diagnostic['error_rt'][0], -802)
        self.assertEqual(self.task.arm_velocity, (0., -self.p['gantry_motion']['max_fb_rpm']))
        self.rt = (-3802, -.25)
        self.step()
        self.assertEqual(self.task.state, 'STOP_PICK')

    def test_stale_image_and_stop_clear_all_outputs(self):
        self.enter_work()
        self.step({2:(200.,0.)})
        self.step()
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.task.stop(self.now,'cancel',False)
        self.assertEqual(self.step(),(0.,0.,0.))
        self.assertEqual(self.task.state,'STOPPED')

    def test_config_validation_and_pid_anti_windup(self):
        validate_coarse_config(self.p)
        self.p['angle_alignment']['timeout_s'] = 3.1
        with self.assertRaisesRegex(ValueError,'3 秒'):
            validate_coarse_config(self.p)
        pid = PID({'kp':1.,'ki':1.,'kd':0.},.1)
        for i in range(20):
            pid.update(100.,i*.1)
        self.assertEqual(pid.integral,0.)
        self.assertEqual(pid.update(0.,2.),0.)

    def test_ring_timeout_stops_then_records_and_places_without_claiming_alignment(self):
        self.task._enter('ALIGN_RING', 0.)
        self.step({2:(130.,120.)}, dt=.125)
        started = self.now
        self.assertTrue(self.task.servo_enabled)
        self.step({2:(130.,120.)}, dt=4.875)
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.step({2:(130.,120.)}, dt=.125)
        self.assertEqual(self.now-started, 5.)
        self.assertEqual(self.task.state, 'STOP_PLACE')
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.assertEqual(self.task.diagnostic['alignment_outcome'], 'TIMEOUT_SKIPPED')
        self.assertIn('未确认达标', self.task.reason)
        self.assertEqual(self.task.placements, {})
        self.step(dt=.25, idle=False)
        self.assertEqual(self.task.state, 'STOP_PLACE')
        self.rt = (1234., .25)
        self.step()
        self.assertEqual(self.task.state, 'PLACE')
        self.assertEqual(self.task.placements[2], self.rt)
        self.step(arm=True)
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.step({3:(130.,120.)})
        self.assertTrue(self.task.servo_enabled)
        self.assertEqual(self.task.diagnostic['alignment_elapsed_s'], 0.)

    def test_ring_timer_starts_on_target_and_survives_tolerance_and_target_loss(self):
        self.task._enter('ALIGN_RING', 0.)
        self.step(rings={}, dt=10.)
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.assertIsNone(self.task._alignment_started)
        self.step({2:(130.,120.)}, dt=.125)
        started = self.now
        for point in ((100.,100.), (103.,100.), (100.,100.), (103.,100.)):
            self.step({2:point}, dt=1.)
        self.step(rings={}, dt=1.)
        self.assertEqual(self.now-started, 5.)
        self.assertEqual(self.task.state, 'STOP_PLACE')
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.assertIsNone(self.task.arm_request)

    def test_timeout_stops_outputs_even_on_repeated_camera_frame(self):
        for state, output in (('ALIGN_RING', 'arm_velocity'), ('CENTER_RING2', 'velocity')):
            with self.subTest(state=state):
                self.task._enter(state, self.now)
                self.step({2:(130.,120.)}, dt=.125)
                self.assertTrue(any(getattr(self.task, output)))
                self.step({2:(130.,120.)}, dt=4.875)
                frozen = Observation('ring', self.sequence, self.now, (201,201), {2:(130.,120.)})
                self.now += .125
                self.task.tick(self.now, Pose(0.,0.,0.,self.now), frozen)
                self.assertEqual(self.task.state, 'STOP_PLACE' if state == 'ALIGN_RING' else state)
                self.assertFalse(any(getattr(self.task, output)))
                self.assertFalse(self.task.servo_enabled)

    def test_base_timeout_waits_for_stopped_fresh_feedback_before_continuing(self):
        self.task._enter('CENTER_RING2', 0.)
        self.step({2:(130.,120.)}, dt=.125)
        self.vx = .1
        self.step({2:(130.,120.)}, dt=5.)
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        self.step({2:(130.,120.)}, dt=.25)
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.vx = 0.
        self.step({2:(130.,120.)})
        stopped_at = self.now
        self.now += .25
        self.sequence += 1
        obs = Observation('ring', self.sequence, self.now, (201,201), {2:(130.,120.)})
        self.task.tick(self.now, Pose(0.,0.,0.,stopped_at), obs)
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.step({2:(130.,120.)})
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.assertIsNone(self.task.reference)

    def test_base_pid_timeout_with_target_lost_still_finishes_stop_handoff(self):
        self.task._enter('CENTER_RING2', self.now)
        self.step({2:(130.,120.)})
        self.step({}, dt=self.p['base_servo']['timeout_s'])
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        self.step({}, dt=.25)
        self.assertEqual(self.task.state, 'ALIGN_RING')

    def test_ring_timeout_configuration_requires_positive_finite_seconds(self):
        for section in ('base_servo', 'gantry_servo'):
            self.assertEqual(self.p[section]['timeout_s'], 5.)
            for value in (0, -1., float('nan'), float('inf'), None):
                with self.subTest(section=section, value=value):
                    self.p[section]['timeout_s'] = value
                    with self.assertRaisesRegex(ValueError, section+'.timeout_s'):
                        validate_coarse_config(self.p)
            self.p[section]['timeout_s'] = 5.


if __name__ == '__main__':
    unittest.main()
