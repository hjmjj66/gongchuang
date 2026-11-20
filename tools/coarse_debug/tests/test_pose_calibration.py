"""Calibrate actual axes with pure tracking, then approach before visual refinement."""

from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

from agv_perception.coarse_control import CoarseTask
from agv_perception.gantry_presets import DEFAULTS, validate_presets
from agv_perception.storage_control import StorageTask
from agv_perception.work_control import GantryPose, Observation, Pose
from coarse_debug.calibration import save_calibration
from coarse_debug.config import read_config
from coarse_debug.pose_calibration import PoseCalibrationTracking, ReferenceAdjustment
from coarse_debug.task import DebugTask, StorageDebugTask


ROOT = Path(__file__).resolve().parents[3]
DEBUG = Path(__file__).resolve().parents[1]/'config/debug.yaml'


class PoseCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.config = read_config(DEBUG)
        for area in ('coarse', 'storage'):
            p = self.config[area]
            p['vision'].update(target_uv_px=[100.,100.], stack_target_uv_px=[120.,110.])
            p['gantry_presets']['positions'] = {'1':dict(fb_pulses=30,yaw_rad=.3),
                                               '3':dict(fb_pulses=40,yaw_rad=-.4)}

    def test_adjust_requires_real_work_angle_and_ring2_then_stops_without_tracking(self):
        task = ReferenceAdjustment(self.config['coarse'], 'adjust', 0.)
        self.assertEqual(task.arm_request['cmd'], 3)
        arm = SimpleNamespace(command_id=task.arm_request['command_id'],state='COMPLETED',idle=True,pending=False)
        task.tick(.1, Pose(0.,0.,0.,.1), arm=arm)
        self.assertEqual(task.state, 'ALIGN_ENTRY')
        for i in range(1,20):
            now=.1+i*.05
            obs=Observation('ring',i,now,(201,201),{1:(20.,100.),2:(100.,100.),3:(180.,100.)})
            task.tick(now,Pose(0.,0.,0.,now),obs,arm,GantryPose(10.,0.,now))
            if task.finished:
                break
        self.assertEqual(task.entry_alignment,'ALIGNED')
        self.assertEqual(task.state,'DONE')
        self.assertEqual(task.reference,(0.,0.))
        self.assertEqual(task.arm_velocity,(0.,0.))
        self.assertIsNone(task.arm_request)
        failed=ReferenceAdjustment(self.config['coarse'],'bad-angle',0.)
        failed._enter('ALIGN_ENTRY',.1)
        with self.assertRaisesRegex(RuntimeError,'纠偏未达标'):
            failed.tick(4.,Pose(0.,0.,0.,4.))

    def test_calibration_ignores_presets_and_keeps_qualification_as_diagnostic(self):
        task = PoseCalibrationTracking(self.config['coarse'], 'track', 3, 0., 'coarse')
        for i in range(1, 5):
            now = i*.05
            obs = Observation('ring', i, now, (201,201), {3:(100.,100.)})
            task.tick(now, observation=obs)
            self.assertEqual(task.arm_velocity, (0.,0.))
            self.assertEqual(task.velocity, (0.,0.,0.))
            task.tick(now+.001, observation=obs)
            self.assertEqual(task.qualified_frames, i)
        with self.assertRaisesRegex(ValueError, '不执行取放'):
            task.request_action(.21)

    def test_save_captures_click_time_position_without_convergence_or_idle(self):
        task = PoseCalibrationTracking(self.config['coarse'], 'track', 3, 0., 'coarse')
        task.tick(.05, observation=Observation('ring', 1, .05, (201,201), {3:(150.,140.)}))
        self.assertEqual(task.qualified_frames, 0)
        self.assertTrue(task.servo_enabled)
        task.request_save(GantryPose(754, -.66523, .05))
        self.assertEqual(task.sample, dict(fb_pulses=754, yaw_rad=-.66523))
        self.assertTrue(task.finished)
        self.assertFalse(task.servo_enabled)
        self.assertEqual(task.arm_velocity, (0.,0.))
        task.tick(.1, gantry=GantryPose(760, -.67, .1))
        self.assertEqual(task.sample, dict(fb_pulses=754, yaw_rad=-.66523))

    def test_save_without_tracking_or_after_manual_or_fault_stop(self):
        for area in ('coarse', 'storage'):
            for stopped in (None, False, True):
                with self.subTest(area=area, stopped=stopped):
                    task = PoseCalibrationTracking(self.config[area], 'save', 1, 0., area)
                    if stopped is not None:
                        task.stop(.1, '反馈故障' if stopped else '网页停止', failed=stopped)
                    task.request_save(GantryPose(-938, -.6427, 1.99))
                    self.assertEqual(task.sample, dict(fb_pulses=-938, yaw_rad=-.6427))
                    self.assertFalse(task.servo_enabled)
                    self.assertEqual(task.reason, '')
                    self.assertTrue(task.finished)

    def test_formal_and_debug_coarse_and_both_storage_visits_approach_then_use_fresh_vision(self):
        tasks=[CoarseTask(self.config['coarse'],'formal',(5,1,6),(3,2,1),0.),
               DebugTask(self.config['coarse'],'debug',(3,2,1),0.)]
        for visit in (1,2):
            tasks.extend([StorageTask(self.config['storage'],'formal-storage',(6,5,1),(3,2,1),0.,
                           'STORAGE_PLACE' if visit==1 else 'STORAGE_STACK',{6:3,5:2,1:1}),
                          StorageDebugTask(self.config['storage'],'debug-storage',
                           (3,2,1) if visit==1 else (6,5,1),0.,visit,{6:3,5:2,1:1})])
        for task in tasks:
            with self.subTest(task=type(task).__name__,operation=getattr(task,'operation','coarse')):
                stack=getattr(task,'operation','')=='STORAGE_STACK'
                next_state='ALIGN_MATERIAL' if stack else 'ALIGN_RING'
                task._enter(next_state,.1)
                self.assertEqual(task.state,'PREPOSITION')
                pose=Pose(0.,0.,0.,.15)
                task.tick(.15,pose,gantry=GantryPose(0.,0.,.15))
                self.assertLess(task.arm_velocity[0],0.)
                self.assertGreater(task.arm_velocity[1],0.)
                self.assertEqual(task.velocity,(0.,0.,0.))
                task.tick(.2,pose,gantry=GantryPose(40.,-.4,.2))
                self.assertEqual(task.state,'STOP_PREPOSITION')
                idle=SimpleNamespace(idle=True,pending=False)
                task.tick(.4,pose,arm=idle,gantry=GantryPose(40.,-.4,.4))
                self.assertEqual(task.state,next_state)
                kind='material' if stack else 'ring'
                old=Observation(kind,1,.39,(201,201),{3:(100.,100.)},((6,(120.,110.)),))
                task.tick(.41,pose,old,idle,GantryPose(40.,-.4,.41))
                self.assertEqual(task.count,0)
                for i in range(4):
                    now=.45+i*.05
                    point=(120.,110.) if stack else (100.,100.)
                    obs=Observation(kind,i+2,now,(201,201),{3:point},((6,point),))
                    task.tick(now,pose,obs,idle,GantryPose(41.,-.41,now))
                self.assertIn(task.state,('STOP_PLACE','PLACE','STACK'))

    def test_ring2_returns_to_current_work_pose_and_unset_presets_keep_visual_fallback(self):
        task=CoarseTask(self.config['coarse'],'coarse',(1,2,3),(2,1,3),0.)
        task._enter('CENTER_RING2',.1)
        task.tick(.15,Pose(0.,0.,0.,.15),gantry=GantryPose(12,.02,.15))
        task._enter('ALIGN_RING',.2)
        self.assertEqual(task.preset_move.target,dict(fb_pulses=12,yaw_rad=.02))
        task.p['gantry_presets']['positions']['1']=None
        task.item=1
        task._enter('ALIGN_RING',.3)
        self.assertEqual(task.state,'ALIGN_RING')

    def test_213_places_approach_each_preset_before_next_ring_is_visible(self):
        tasks = [CoarseTask(self.config['coarse'], 'formal-213', (2,1,3), (2,1,3), 0.),
                 StorageTask(self.config['storage'], 'storage-213', (2,1,3), (2,1,3),
                             0., 'STORAGE_PLACE', {})]
        for execution in ('simulated', 'real'):
            tasks.extend([DebugTask(self.config['coarse'], 'debug-213', (2,1,3), 0.,
                                    execution=execution),
                          StorageDebugTask(self.config['storage'], 'storage-debug-213',
                                           (2,1,3), 0., 1, {}, execution=execution)])
        for task in tasks:
            with self.subTest(task=type(task).__name__, execution=getattr(task, 'execution', 'real')):
                now, sequence = 0., 0
                axes = (0, 1.)
                task.work_gantry = dict(fb_pulses=10, yaw_rad=0.)

                def step(rings, *, dt=.05, complete=False):
                    nonlocal now, sequence
                    now += dt
                    sequence += 1
                    arm = SimpleNamespace(command_id='', state='IDLE', idle=True, pending=False)
                    if complete:
                        arm.command_id = task.arm_request['command_id']
                        arm.state = 'COMPLETED'
                    obs = Observation('ring', sequence, now, (201,201), rings)
                    task.tick(now, Pose(0.,0.,0.,now), obs, arm, GantryPose(*axes,now))

                task._enter('ALIGN_RING', now)
                for item, ring in enumerate((2,1,3)):
                    previous_view = {} if item == 0 else {(2,1,3)[item-1]:(100.,100.)}
                    target = task.work_gantry if ring == 2 else task.p['gantry_presets']['positions'][str(ring)]
                    self.assertEqual(task.item, item)
                    self.assertEqual(task.state, 'PREPOSITION')
                    self.assertEqual(task.preset_move.target, target)
                    step(previous_view)
                    self.assertEqual(task.state, 'PREPOSITION')
                    self.assertTrue(task.servo_enabled)
                    self.assertTrue(any(task.arm_velocity))
                    self.assertIsNone(task._alignment_started)
                    axes = (target['fb_pulses'], target['yaw_rad'])
                    step(previous_view)
                    self.assertEqual(task.state, 'STOP_PREPOSITION')
                    step(previous_view, dt=.2)
                    self.assertEqual(task.state, 'ALIGN_RING')
                    step(previous_view, dt=.6)
                    self.assertEqual(task.state, 'ALIGN_RING')
                    self.assertIsNone(task._alignment_started)
                    self.assertIsNone(task.arm_request)
                    for _ in range(4):
                        step({ring:(100.,100.)})
                    if task.state == 'STOP_PLACE':
                        step({}, dt=.2)
                        self.assertEqual(task.placements[ring], axes)
                    self.assertEqual(task.state, 'PLACE')
                    if getattr(task, 'execution', 'real') == 'simulated':
                        step({}, dt=1.01)
                    else:
                        axes = (10, 0.)
                        step({}, complete=True)
                    if item < 2:
                        self.assertEqual(task.state, 'PREPOSITION')
                        self.assertIsNone(task.arm_request)
                self.assertEqual([row[3] for row in task.confirmed], [2,1,3])
                self.assertEqual(task.state, 'STOW' if isinstance(task, StorageTask) else 'RETURN_ITEM')

    def test_preposition_requires_actual_feedback_has_timeout_and_cancel_stops(self):
        task=CoarseTask(self.config['coarse'],'coarse',(1,2,3),(1,2,3),0.)
        task._enter('ALIGN_RING',.1)
        pose=Pose(0.,0.,0.,.2)
        with self.assertRaisesRegex(RuntimeError,'新鲜位置反馈'):
            task.tick(.2,pose)
        task.tick(40.,pose,gantry=GantryPose(0.,0.,40.))
        self.assertEqual(task.state, 'STOP_PREPOSITION')
        self.assertEqual(task.arm_velocity, (0.,0.))
        task.tick(40.2, pose, arm=SimpleNamespace(idle=True,pending=False),
                  gantry=GantryPose(0.,0.,40.2))
        self.assertEqual(task.state, 'ALIGN_RING')
        self.assertIn('预定位PID超时', task.reason)
        task.stop(.3,'stop',False)
        self.assertEqual(task.arm_velocity,(0.,0.))
        self.assertFalse(task.servo_enabled)

    def test_preposition_time_does_not_consume_ring_pid_budget(self):
        self.config['coarse']['gantry_motion']['timeout_s'] = 30.
        task=CoarseTask(self.config['coarse'],'coarse',(1,2,3),(1,2,3),0.)
        task._enter('ALIGN_RING', 0.)
        task.tick(6., Pose(0.,0.,0.,6.), gantry=GantryPose(0.,0.,6.))
        self.assertEqual(task.state, 'PREPOSITION')
        task.tick(7., Pose(0.,0.,0.,7.), gantry=GantryPose(30.,.3,7.))
        self.assertEqual(task.state, 'STOP_PREPOSITION')
        idle=SimpleNamespace(idle=True,pending=False)
        task.tick(7.25, Pose(0.,0.,0.,7.25), arm=idle, gantry=GantryPose(30.,.3,7.25))
        self.assertEqual(task.state, 'ALIGN_RING')
        obs=Observation('ring',1,7.375,(201,201),{1:(130.,120.)})
        task.tick(7.375, Pose(0.,0.,0.,7.375), obs)
        self.assertTrue(task.servo_enabled)
        self.assertEqual(task.diagnostic['alignment_elapsed_s'], 0.)

    def test_two_shared_poses_sync_both_areas_and_rollback_all_files(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            debug=folder/'debug.yaml'
            debug.write_text(DEBUG.read_text())
            config=read_config(debug)
            paths={area:folder/f'{area}.yaml' for area in ('coarse','storage')}
            for area,path in paths.items():
                path.write_text((ROOT/f'ros2_ws/src/agv_perception/config/{area}.yaml').read_text())
            before={area:yaml.safe_load(path.read_text()) for area,path in paths.items()}
            # An old independent storage value must be replaced by the shared group.
            config['coarse']['gantry_presets']['positions']['3']=dict(fb_pulses=44,yaw_rad=-.4)
            for ring in (1,3):
                point=dict(fb_pulses=ring*10,yaw_rad=-ring*.1)
                config=save_calibration(debug,config,'shared_pose',point,paths,ring=ring)
                positions=config['coarse']['gantry_presets']['positions']
                self.assertEqual(positions[str(ring)],point)
                if ring==1:
                    self.assertEqual(positions['3'],dict(fb_pulses=44,yaw_rad=-.4))
                for area in paths:
                    self.assertEqual(config[area]['gantry_presets']['positions'],positions)
                    before[area][area]['gantry_presets']['positions']=deepcopy(positions)
                    self.assertEqual(yaml.safe_load(paths[area].read_text()),before[area])
                    self.assertEqual(read_config(debug)[area]['gantry_presets'],config[area]['gantry_presets'])
            originals={path:path.read_bytes() for path in (debug,*paths.values())}
            import os
            replace=os.replace
            def fail_formal(src,dst):
                if dst==paths['storage']:
                    raise OSError('模拟写失败')
                replace(src,dst)
            with patch('coarse_debug.persistence.os.replace',side_effect=fail_formal):
                with self.assertRaises(OSError):
                    save_calibration(debug,config,'shared_pose',dict(fb_pulses=99,yaw_rad=-1.),paths,ring=1)
            for path,content in originals.items():
                self.assertEqual(path.read_bytes(),content)
        invalid=deepcopy(DEFAULTS)
        invalid['positions']['1']=dict(fb_pulses=1.5,yaw_rad=.1)
        with self.assertRaises(ValueError):
            validate_presets(invalid)

    def test_legacy_files_add_shared_presets_and_keep_area_specific_parameters(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            config=read_config(DEBUG)
            config['storage'].pop('gantry_presets')
            debug=folder/'debug.yaml'
            debug.write_text(yaml.safe_dump(config,allow_unicode=True))
            paths={}
            before={}
            for area in ('coarse','storage'):
                original=yaml.safe_load((ROOT/f'ros2_ws/src/agv_perception/config/{area}.yaml').read_text())
                original[area].pop('gantry_presets')
                source,installed=folder/f'{area}-source.yaml',folder/f'{area}-installed.yaml'
                source.write_text(yaml.safe_dump(original,allow_unicode=True))
                original[area]['arm']['work_pose_cmd']=2
                installed.write_text(yaml.safe_dump(original,allow_unicode=True))
                paths[area]=[source,installed]
                before[area]=original
            point=dict(fb_pulses=20,yaw_rad=-.2)
            result=save_calibration(debug,config,'shared_pose',point,paths,ring=3)
            for area,destinations in paths.items():
                self.assertEqual(result[area]['gantry_presets']['positions']['3'],point)
                for path in destinations:
                    written=yaml.safe_load(path.read_text())[area]
                    self.assertEqual(written['gantry_presets']['positions'],result['coarse']['gantry_presets']['positions'])
                self.assertEqual(yaml.safe_load(destinations[1].read_text())[area]['arm']['work_pose_cmd'],2)

    def test_presets_accept_signed_pulses_and_reject_old_mm_points(self):
        from agv_perception.gantry_presets import PresetMove
        config = deepcopy(DEFAULTS)
        config['positions']['1'] = dict(fb_pulses=-3802, yaw_rad=-.25)
        validate_presets(config)
        motion = self.config['coarse']['gantry_motion']
        move = PresetMove(motion, config['positions']['1'], 0., .2)
        good, velocity, diagnostic = move.update(.1, GantryPose(-3000, -.25, .1))
        self.assertFalse(good)
        self.assertEqual(velocity, (0., -motion['max_fb_rpm']))
        self.assertEqual(diagnostic['error_rt'], (-802, 0.))
        config['positions']['1'] = dict(fb_mm=100., yaw_rad=-.25)
        with self.assertRaisesRegex(ValueError, '旧毫米点位'):
            validate_presets(config)
        for value in (True, 1.5, -2147483649, 2147483648):
            config['positions']['1'] = dict(fb_pulses=value, yaw_rad=-.25)
            with self.assertRaises(ValueError):
                validate_presets(config)


if __name__=='__main__':
    unittest.main()
