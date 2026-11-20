from motion_fixture import copy_debug_config
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from agv_perception.coarse_control import GantryPose, Observation, Pose
from coarse_debug.config import FIELDS, get_value, parse_order, read_config, save_parameters
from coarse_debug.task import DebugTask

CONFIG = Path(__file__).parents[1]/'config/debug.yaml'


class DebugTests(unittest.TestCase):
    def setUp(self):
        self.config = read_config(CONFIG)
        self.p = deepcopy(self.config['coarse'])
        self.p.pop('gantry_presets', None)  # These task cases exercise visual fallback.
        self.p['vision']['target_uv_px'] = [100.,100.]
        self.task = DebugTask(self.p,'debug-test',(2,1,3),0.)
        self.x = 0.
        self.now, self.sequence = 0., 0
        self.rt = (0.,0.)
        self.actions = []

    def step(self,rings=None,dt=.05,arm=False):
        self.now += dt
        self.sequence += 1
        obs = None if rings is None else Observation('ring',self.sequence,self.now,(201,201),rings)
        status = SimpleNamespace(command_id='',state='IDLE',idle=True,pending=False)
        if arm:
            self.actions.append(self.task.arm_request['cmd'])
            status.command_id = self.task.arm_request['command_id']
            status.state = 'COMPLETED'
        self.assertNotEqual(self.task.vision_kind,'material')
        return self.task.tick(self.now,Pose(self.x,0.,0.,self.now),obs,status,GantryPose(*self.rt,self.now))

    def align(self):
        state = self.task.state
        for _ in range(15):
            self.step({1:(20.,100.),2:(100.,100.),3:(180.,100.)})
            if self.task.state != state:
                return
        self.fail(f'Alignment did not complete: {state}')

    def simulate_operation(self,op):
        self.assertEqual(self.task.state,op)
        self.assertIsNone(self.task.arm_request)
        self.step(dt=.99)
        self.assertEqual(self.task.state,op)
        self.step(dt=.02)
        if op == 'PLACE':
            self.assertIsNone(self.task.arm_request)
            return
        self.assertEqual(self.task.state,'DEBUG_RESTORE_WORK')
        self.assertEqual(self.task.arm_request['cmd'],3)
        self.rt = (0.,0.)
        self.step(arm=True)

    def test_full_213_work_pose_without_lower_stow_and_six_simulated_operations(self):
        self.assertEqual(self.task.arm_request['cmd'],3)
        self.step(arm=True)
        self.align()
        self.assertEqual(self.task.state,'CAPTURE_REFERENCE')
        for _ in range(4): self.step()
        self.assertEqual(self.task.state,'CENTER_RING2')
        self.align()
        for i,ring in enumerate((2,1,3)):
            self.assertEqual(self.task.state,'ALIGN_RING')
            self.assertIsNone(self.task.arm_request)
            self.rt = (10.+i,.1*i)
            for _ in range(4):
                self.step({ring:(100.,100.)})
            self.step(dt=.15)
            self.assertEqual(self.task.placements[ring],self.rt)
            self.simulate_operation('PLACE')
        for ring in (2,1,3):
            self.assertEqual(self.task.state,'RETURN_ITEM')
            self.rt = self.task.placements[ring]
            self.step()
            self.step(dt=.15)
            self.simulate_operation('PICK')
        self.assertEqual(self.task.state,'WORK_EXIT')
        self.step(arm=True)
        self.align()
        self.assertEqual(self.task.state,'STOW')
        self.step(arm=True)
        self.assertEqual(self.task.state,'RETURN_REFERENCE')
        self.x = .08
        self.assertLess(self.step()[0],0.)
        self.x = self.task.reference[0]
        self.step()
        self.step(dt=.15)
        self.assertEqual(self.task.state,'DONE')
        self.assertNotIn(None,self.actions)
        self.assertEqual(self.actions.count(3),5)
        self.assertEqual(self.actions[-1],2)
        self.assertNotIn(6,self.actions)
        self.assertNotIn(5,self.actions)
        self.assertEqual([(op,p) for op,_,_,p in self.task.confirmed],
                         [('PLACE',2),('PLACE',1),('PLACE',3),('PICK',2),('PICK',1),('PICK',3)])

    def test_alignment_timeout_still_requires_high_pose_and_stow(self):
        self.step(arm=True)
        self.assertGreater(self.step({1:(0.,100.),3:(200.,70.)})[2],0.)
        self.step(dt=3.01)
        self.assertEqual(self.task.state,'CAPTURE_REFERENCE')
        self.assertEqual(self.task.entry_alignment,'TIMEOUT_SKIPPED')
        self.task._enter('ALIGN_EXIT',self.now)
        self.step(dt=3.01)
        self.assertEqual(self.task.state,'STOW')
        self.assertEqual(self.task.arm_request['cmd'],2)
        self.assertEqual(self.task.final_alignment,'TIMEOUT_SKIPPED')

    def test_saved_gantry_values_reload_and_invalid_save_keeps_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'debug.yaml'
            copy_debug_config(path)
            values = {f['path']:get_value(self.config,f['path']) for f in FIELDS}
            values['gantry_servo.u_pid.kp']=.0004
            values['base_servo.timeout_s']=4.5
            values['gantry_servo.timeout_s']=6.
            values['vision.target_uv_px']=[110.,120.]
            previous_gain=self.config['coarse']['gantry_servo']['u_pid']['kp']
            saved=save_parameters(path,self.config,values)
            self.assertEqual(read_config(path),saved)
            self.assertEqual(self.config['coarse']['gantry_servo']['u_pid']['kp'],previous_gain)
            self.assertEqual(saved['storage']['gantry_servo'],saved['coarse']['gantry_servo'])
            self.assertEqual(saved['storage']['vision'],self.config['storage']['vision'])
            self.assertEqual(saved['storage']['exit_motion'],self.config['storage']['exit_motion'])
            self.assertEqual(saved['raw'],self.config['raw'])
            self.assertEqual(saved['coarse']['base_servo']['timeout_s'],4.5)
            self.assertEqual(saved['coarse']['gantry_servo']['timeout_s'],6.)
            previous=path.read_bytes()
            for bad in (None,float('nan'),-1,0):
                invalid=deepcopy(values)
                invalid['gantry_motion.max_fb_rpm']=bad
                with self.assertRaises(ValueError):
                    save_parameters(path,saved,invalid)
                self.assertEqual(path.read_bytes(),previous)
            for field in ('base_servo.timeout_s', 'gantry_servo.timeout_s'):
                for bad in (None, float('nan'), -1, 0):
                    invalid=deepcopy(values)
                    invalid[field]=bad
                    with self.assertRaises(ValueError):
                        save_parameters(path,saved,invalid)
                    self.assertEqual(path.read_bytes(),previous)
            values['angle_alignment.timeout_s']=3.01
            with self.assertRaises(ValueError):
                save_parameters(path,saved,values)
            self.assertEqual(path.read_bytes(),previous)

    def test_213_ring3_return_timeout_continues_in_simulated_and_real_modes(self):
        for execution in ('simulated', 'real'):
            with self.subTest(execution=execution):
                self.task = DebugTask(self.p, '213-timeout', (2,1,3), self.now, execution=execution)
                self.task.item = 2
                self.task.placements[3] = (2304, -.39514994621276855)
                self.task.return_started = self.now
                self.task._enter('RETURN_ITEM', self.now)
                self.rt = (2316, -.39543747901916504)
                self.step(dt=self.p['gantry_motion']['timeout_s'])
                self.assertEqual(self.task.state, 'STOP_PICK')
                self.assertEqual(self.task.arm_velocity, (0.,0.))
                self.step(dt=.2)
                self.assertEqual(self.task.state, 'PICK')
                if execution == 'simulated':
                    self.step(dt=1.01)
                    self.assertEqual(self.task.state, 'DEBUG_RESTORE_WORK')
                else:
                    self.assertEqual(self.task.arm_request['cmd'], 5)
                self.step(arm=True)
                self.assertEqual(self.task.state, 'WORK_EXIT')

    def test_order_uses_exactly_ring_ids_123(self):
        self.assertEqual(parse_order('213'),(2,1,3))
        for bad in ('2311','112','123 ','',123,[2,1,3],'１２３'):
            with self.assertRaises(ValueError):
                parse_order(bad)



if __name__ == '__main__':
    unittest.main()
