"""Exercise storage's real state machine with fresh camera and action observations."""
import math
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import unittest

from agv_perception.work_control import Observation, Pose
from agv_perception.storage_control import StorageTask, read_storage_config, validate_storage_config


def calibrated_storage_config():
    p = read_storage_config(Path(__file__).parents[1]/'config/storage.yaml')
    p.pop('gantry_presets', None)  # These cases exercise the uncalibrated visual fallback.
    def fill(section):
        for key, value in section.items():
            if isinstance(value, dict):
                fill(value)
            elif value is None:
                section[key] = .1
    fill(p)
    p['vision']['target_uv_px'] = [100.,100.]
    p['vision']['stack_target_uv_px'] = [100.,100.]
    p['base_servo'].update(tolerance_px=2.,settle_frames=2,stop_speed_mps=.01)
    p['gantry_servo'].update(tolerance_px=2.,small_yaw_rad_s=.005,small_fb_rpm=.2)
    p['angle_alignment'].update(slope_tolerance=.02,settle_frames=2)
    p['ring_search'].update(scan_settle_frames=2,duration_s=.3)
    return p


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.p = calibrated_storage_config()
        self.now = 0.
        self.sequence = 0
        self.xy = (1.,2.)
        self.yaw, self.vx = 0., 0.
        self.layer = {1:1,5:2,6:3}
        self.actions = []

    def start(self, operation='STORAGE_STACK'):
        stack = operation == 'STORAGE_STACK'
        self.task = StorageTask(self.p,'storage-test',[5,1,6] if stack else [1,5,6],
                                [2,1,3] if stack else [2,1,3],self.now,operation,self.layer if stack else {})

    def step(self, materials=None, rings=None, arm=False, dt=.05):
        self.now += dt
        self.sequence += 1
        obs = (Observation('material' if materials is not None else 'ring',self.sequence,self.now,
                           (201,201),rings or {},materials or ())
               if materials is not None or rings is not None else None)
        status = None
        if arm:
            self.actions.append(self.task.arm_request['cmd'])
            status = SimpleNamespace(command_id=self.task.arm_request['command_id'],state='COMPLETED')
        return self.task.tick(self.now,Pose(*self.xy,self.yaw,self.now,self.vx),obs,status)

    def begin_stack(self):
        self.start()
        self.step(arm=True)
        self.assertEqual(self.task.state,'SCAN_FIRST_LAYER')
        self.assertTrue(self.task.base_locked)

    def scan_all(self):
        for _ in range(2):
            self.step(materials=((1,(20.,90.)),(5,(110.,90.)),(6,(180.,90.))))
        self.assertEqual(self.task.state,'SCAN_RESTORE_WORK')
        self.assertEqual(self.task.arm_request['cmd'],3)
        self.step(arm=True)
        self.assertEqual(self.task.state,'ALIGN_MATERIAL')

    def align_and_place(self, color=None, ring=None):
        args = dict(materials=((color,(100.,100.)),)) if color else dict(rings={ring:(100.,100.)})
        for _ in range(4):
            self.step(**args)
        self.assertEqual(self.task.state,'STACK' if color else 'PLACE')
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.assertEqual(self.task.arm_request['cmd'],7 if color else 6)
        self.task.tick(self.now,Pose(*self.xy,0.,self.now),arm=SimpleNamespace(command_id='old',state='COMPLETED'))
        self.assertIn(self.task.state,('PLACE','STACK'))
        self.step(arm=True)

    def finish_stow(self):
        self.assertEqual(self.task.state,'STOW')
        self.assertEqual(self.task.arm_request['cmd'],2)
        self.assertFalse(self.task.finished)
        self.step(arm=True)
        if self.task.operation == 'STORAGE_PLACE':
            self.assertEqual(self.task.state,'RETURN_REFERENCE')
            self.assertFalse(self.task.base_locked)
            self.xy = (self.task.reference[0]+.08,self.task.reference[1]-.04)
            self.assertLess(self.step()[0],0.)
            self.assertGreater(self.task.velocity[1],0.)
            self.xy = self.task.reference
            self.step()
            self.step(dt=.15)
        self.assertEqual(self.task.state,'DONE')
        self.assertEqual(self.task.returned, self.task.operation == 'STORAGE_PLACE')
        self.assertEqual(self.task.final_alignment,'NOT_RUN')
        self.assertEqual(self.task.placements,{})

    def test_first_visit_calibrates_ring2_places_high_then_stows_and_returns_p0(self):
        self.start('STORAGE_PLACE')
        self.assertFalse(self.task.base_locked)
        self.step(arm=True)
        rings={1:(20.,100.),2:(100.,100.),3:(180.,100.)}
        for _ in range(20):
            if self.task.state == 'ALIGN_RING':
                break
            self.step(rings=rings)
        self.assertEqual(self.task.state,'ALIGN_RING')
        self.assertEqual(self.task.reference,self.xy)
        self.assertTrue(self.task.base_locked)
        self.assertEqual(self.actions,[3])
        self.assertFalse(self.task.confirmed)
        for ring in (2,1,3):
            self.align_and_place(ring=ring)
        self.assertEqual(self.task.confirmed,[('PLACE',1,1,2),('PLACE',2,5,1),('PLACE',3,6,3)])
        self.finish_stow()
        self.assertEqual(self.actions,[3,6,6,6,2])

    def test_first_visit_captures_p0_before_translation_and_returns_after_stow(self):
        self.start('STORAGE_PLACE')
        self.step(arm=True)
        rings = {1:(20.,100.),2:(100.,100.),3:(180.,100.)}
        while self.task.state == 'ALIGN_ENTRY': self.step(rings=rings)
        self.assertEqual(self.task.state, 'CAPTURE_REFERENCE')
        self.assertIsNone(self.task.reference)
        for _ in range(4): self.step()
        self.assertEqual(self.task.reference, (1.,2.))
        self.xy = (1.1,1.95)
        while self.task.state == 'CENTER_RING2': self.step(rings=rings)
        self.assertEqual(self.task.reference, (1.,2.))
        self.task._enter('STOW',self.now)
        self.yaw = math.pi/2
        self.assertEqual(self.step(),(0.,0.,0.))
        self.step(arm=True)
        self.assertFalse(self.task.base_locked)
        self.task.tick(self.now+.01,Pose(99.,99.,0.,self.now-.01))
        self.assertEqual(self.task.velocity,(0.,0.,0.))
        vx,vy,wz = self.step()
        self.assertGreater(vx,0.)
        self.assertGreater(vy,0.)
        self.assertEqual(wz,0.)
        self.xy,self.vx = self.task.reference,.02
        self.assertEqual(self.step(),(0.,0.,0.))
        self.step(dt=.2)
        self.assertFalse(self.task.finished)
        self.vx = 0.
        self.step()
        self.assertTrue(self.task.returned)
        self.assertEqual(self.task.state,'DONE')

    def test_first_visit_can_stow_and_stop_in_place_for_navigation(self):
        self.start('STORAGE_PLACE')
        self.task.defer_reference_return = True
        self.task.reference = self.xy
        self.xy = (1.2, 1.7)
        self.task._enter('STOW', self.now)
        self.assertEqual(self.step(), (0.,0.,0.))
        self.assertFalse(self.task.finished)
        self.step(arm=True)
        self.assertEqual(self.task.state, 'STOP_EXIT')
        self.assertTrue(self.task.base_locked)
        self.step()
        self.step(dt=.15)
        self.assertEqual(self.task.state, 'DONE')
        self.assertTrue(self.task.reference_return_deferred)
        self.assertFalse(self.task.returned)
        self.assertEqual(self.task.final_alignment, 'NOT_RUN')
        self.assertEqual(self.xy, (1.2, 1.7))

    def test_return_timeout_cancel_and_stack_never_moves(self):
        self.start('STORAGE_PLACE')
        self.task.reference = self.xy
        self.task._enter('STOW',self.now)
        self.step(arm=True)
        self.xy = (1.1,2.1)
        self.step()
        self.step(dt=16.)
        self.assertEqual(self.task.state, 'STOP_EXIT')
        self.step()
        self.step(dt=.2)
        self.assertEqual(self.task.state, 'DONE')
        self.assertTrue(self.task.reference_return_timed_out)
        self.assertFalse(self.task.returned)
        self.task.stop(self.now,'cancel',False)
        self.assertEqual(self.task.velocity,(0.,0.,0.))
        self.start('STORAGE_STACK')
        self.task._enter('STOW',self.now)
        self.step(arm=True)
        self.assertEqual(self.task.state,'DONE')
        self.assertEqual(self.task.velocity,(0.,0.,0.))
        self.assertFalse(self.task.returned)

    def test_stack_pid_timeout_continues_even_after_target_loss(self):
        self.begin_stack()
        self.scan_all()
        self.step(materials=((5,(140.,130.)),))
        self.assertTrue(self.task.servo_enabled)
        self.step(materials=(), dt=self.p['gantry_servo']['timeout_s'])
        self.assertEqual(self.task.state, 'STACK')
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.assertEqual(self.task.arm_request['cmd'], 7)
        self.assertIn('未确认达标', self.task.reason)

    def test_second_visit_partial_scan_restores_work_then_uses_current_color_centers(self):
        self.begin_stack()
        for _ in range(2):
            self.assertEqual(self.step(materials=((5,(100.,90.)),)),(0.,0.,0.))
        self.assertEqual(self.task.seen_colors,{5})
        self.assertGreater(self.task.arm_velocity[0],0.)
        for _ in range(2):
            self.step(materials=((1,(20.,90.)),(6,(180.,90.))))
        self.assertEqual(self.task.seen_colors,{1,5,6})
        self.assertEqual(self.task.state,'SCAN_RESTORE_WORK')
        self.assertFalse(self.task.confirmed)
        self.step(arm=True)
        self.step(materials=())  # Ring2 absent: wait, never reuse its scanned center.
        self.assertEqual(self.task.count,0)
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        for color in (5,1,6):
            self.step(materials=((color,(200.,0.)),))
            self.assertLess(self.task.arm_velocity[0],0.)
            self.assertGreater(self.task.arm_velocity[1],0.)
            self.assertEqual(self.task.velocity,(0.,0.,0.))
            self.align_and_place(color=color)
        self.assertEqual(self.task.confirmed,[('STACK',1,5,2),('STACK',2,1,1),('STACK',3,6,3)])
        self.finish_stow()
        self.assertEqual(self.actions,[3,3,7,7,7,2])
        self.assertEqual(self.task.entry_alignment,'NOT_RUN')

    def test_four_new_frames_require_small_normal_outputs_and_loss_resets_gate(self):
        self.begin_stack(); self.scan_all()
        self.p['gantry_servo']['small_yaw_rad_s']=.00001
        self.step(materials=((5,(101.,100.)),))
        self.assertEqual(self.task.count,0)
        self.step(materials=((5,(100.,100.)),))
        frozen=Observation('material',self.sequence,self.now,(201,201),{},((5,(100.,100.)),))
        self.task.tick(self.now+.01,Pose(*self.xy,0.,self.now),frozen)
        self.assertEqual(self.task.count,1)
        self.step(materials=())
        self.assertEqual(self.task.count,0)
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        for _ in range(3):
            self.step(materials=((5,(100.,100.)),))
        self.assertEqual(self.task.state,'ALIGN_MATERIAL')
        self.step(materials=((5,(100.,100.)),))
        self.assertEqual(self.task.state,'STACK')

    def test_initial_search_waits_until_first_sighting_then_pid_timeout_continues(self):
        self.begin_stack(); self.scan_all()
        self.task.item=1  # Color1 lives on the left of the known WORK pose.
        self.task._enter('ALIGN_MATERIAL',self.now)
        self.step(materials=())
        self.assertGreater(self.task.arm_velocity[0],0.)
        for _ in range(20):
            self.step(materials=())
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        for _ in range(700):
            self.step(materials=())
            self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertEqual(self.task.state,'ALIGN_MATERIAL')
        self.step(materials=((1,(90.,95.)),))
        self.assertTrue(self.task.servo_enabled)
        self.step(materials=(), dt=self.p['gantry_servo']['timeout_s'])
        self.assertEqual(self.task.state, 'STACK')

    def test_scan_is_bounded_both_sides_and_waits_without_business_deadline(self):
        self.begin_stack()
        directions=set()
        for _ in range(700):
            self.step(materials=((5,(100.,90.)),))
            if self.task.arm_velocity[0]:
                directions.add(1 if self.task.arm_velocity[0]>0 else -1)
        self.assertEqual(directions,{-1,1})
        self.assertEqual(self.task.scan_explored,{1,6})
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertFalse(self.task.confirmed)
        self.assertEqual(self.task.state,'SCAN_FIRST_LAYER')
        self.task.stop(self.now,'cancel',False)
        self.assertEqual(self.step(),(0.,0.,0.))
        self.assertFalse(self.task.servo_enabled)

    def test_scanning_counts_new_frames_only_and_search_stops_between_images(self):
        self.begin_stack()
        all_colors=((1,(20.,100.)),(5,(100.,100.)),(6,(180.,100.)))
        self.step(materials=all_colors)
        old=Observation('material',self.sequence,self.now,(201,201),{},all_colors)
        self.task.tick(self.now+.01,Pose(*self.xy,0.,self.now),old)
        self.assertFalse(self.task.seen_colors)
        self.step(materials=())
        self.assertTrue(self.task.servo_enabled)
        self.task.tick(self.now+.31,Pose(*self.xy,0.,self.now),
                       Observation('material',self.sequence,self.now,(201,201),{},()))
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.assertIn(1,self.task.scan_explored)

    def test_stale_camera_waits_but_action_failure_still_raises(self):
        self.begin_stack()
        self.step(materials=())
        for delay in (.6, 3600.):
            self.step(dt=delay)
            self.assertFalse(self.task.finished)
            self.assertEqual(self.task.arm_velocity,(0.,0.))
            self.assertFalse(self.task.servo_enabled)
        self.start('STORAGE_PLACE')
        with self.assertRaisesRegex(RuntimeError,'rejected'):
            self.task.tick(self.now,Pose(*self.xy,0.,self.now),arm=SimpleNamespace(
                command_id=self.task.arm_request['command_id'],state='FAILED',reason='rejected'))

    def test_angle_timeout_then_ring2_wait_and_configuration_calibration(self):
        self.start('STORAGE_PLACE'); self.step(arm=True)
        self.step(rings={},dt=3.)
        self.assertEqual(self.task.state,'CAPTURE_REFERENCE')
        for _ in range(4): self.step()
        self.assertEqual(self.task.state,'CENTER_RING2')
        self.assertEqual(self.task.entry_alignment,'TIMEOUT_SKIPPED')
        self.assertEqual(self.actions,[3])
        self.step(rings={1:(20.,100.),3:(180.,100.)})
        self.assertEqual(self.task.velocity,(0.,0.,0.))
        self.assertFalse(self.task.servo_enabled)
        validate_storage_config(self.p)
        for path,value in [('timeout_s',3.1)]:
            bad=deepcopy(self.p);bad['angle_alignment'][path]=value
            with self.assertRaises(ValueError): validate_storage_config(bad)
        formal = read_storage_config(Path(__file__).parents[1]/'config/storage.yaml')
        validate_storage_config(formal)
        formal['angle_alignment']['slope_tolerance'] = None
        with self.assertRaisesRegex(ValueError,'未标定'):
            validate_storage_config(formal)

    def test_first_visit_ring_timeout_continues_place_and_resets_for_next_ring(self):
        self.start('STORAGE_PLACE')
        self.task._enter('ALIGN_RING', self.now)
        self.step(rings={2:(130.,120.)}, dt=.125)
        self.assertTrue(self.task.servo_enabled)
        self.step(rings={2:(130.,120.)}, dt=5.)
        self.assertEqual(self.task.state, 'PLACE')
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.assertFalse(self.task.servo_enabled)
        self.assertEqual(self.task.arm_request['cmd'], 6)
        self.assertIn('未确认达标', self.task.reason)
        self.assertFalse(self.task.confirmed)
        self.step(arm=True)
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.step(rings={1:(130.,120.)})
        self.assertTrue(self.task.servo_enabled)
        self.assertEqual(self.task.diagnostic['alignment_elapsed_s'], 0.)

    def test_first_visit_base_timeout_stops_then_proceeds_to_gantry(self):
        self.start('STORAGE_PLACE')
        self.task._enter('CENTER_RING2', self.now)
        self.step(rings={2:(130.,120.)}, dt=.125)
        self.assertNotEqual(self.task.velocity, (0.,0.,0.))
        self.step(rings={2:(130.,120.)}, dt=5.)
        self.assertEqual(self.task.state, 'CENTER_RING2')
        self.assertEqual(self.task.velocity, (0.,0.,0.))
        self.step(rings={2:(130.,120.)}, dt=.25)
        self.assertEqual(self.task.state, 'ALIGN_RING')
        self.assertTrue(self.task.base_locked)

    def test_material_alignment_timeout_continues_stack(self):
        self.begin_stack(); self.scan_all()
        self.step(materials=((5,(130.,120.)),), dt=.125)
        self.step(materials=((5,(130.,120.)),), dt=5.)
        self.assertEqual(self.task.state, 'STACK')
        self.assertFalse(self.task.servo_enabled)
        self.assertEqual(self.task.arm_velocity, (0.,0.))
        self.assertEqual(self.task.arm_request['cmd'], 7)


if __name__=='__main__':
    unittest.main()
