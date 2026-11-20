"""Run the production storage controllers with arm operations replaced by waits."""
from motion_fixture import copy_debug_config
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from agv_perception.coarse_control import Observation, Pose
from coarse_debug.config import area_fields, get_value, parse_task, read_config, save_parameters
from coarse_debug.task import StorageDebugTask

CONFIG = Path(__file__).parents[1]/'config/debug.yaml'


class StorageDebugTests(unittest.TestCase):
    def setUp(self):
        self.config = read_config(CONFIG)
        self.p = deepcopy(self.config['storage'])
        self.p.pop('gantry_presets', None)  # These cases exercise visual alignment without position feedback.
        self.p['vision']['target_uv_px'] = [100.,100.]
        self.p['vision']['stack_target_uv_px'] = [100.,100.]
        self.now = 0.
        self.sequence = 0
        self.xy = (1.,2.)
        self.actions = []

    def step(self, rings=None, materials=None, dt=.05, arm=False):
        from types import SimpleNamespace
        self.now += dt
        self.sequence += 1
        obs = None
        if rings is not None or materials is not None:
            obs = Observation('material' if materials is not None else 'ring',self.sequence,
                              self.now,(201,201),rings or {},materials or ())
        status = None
        if arm:
            self.actions.append(self.task.arm_request['cmd'])
            status = SimpleNamespace(command_id=self.task.arm_request['command_id'],state='COMPLETED')
        self.task.tick(self.now,Pose(*self.xy,0.,self.now),obs,status)
        return self.task.velocity

    def align_item(self, ring=None, color=None):
        args = dict(rings={ring:(100.,100.)}) if ring else dict(materials=((color,(100.,100.)),))
        for _ in range(4): self.step(**args)
        state = 'PLACE' if ring else 'STACK'
        self.assertEqual(self.task.state,state)
        self.assertIsNone(self.task.arm_request)
        self.assertEqual(self.task.arm_velocity,(0.,0.))
        self.step(dt=.99)
        self.assertEqual(self.task.state,state)
        self.step(dt=.02)
        self.assertEqual(self.task.state, 'STOW' if self.task.item == 3 else self.task._item_state())
        if self.task.item < 3:
            self.assertIsNone(self.task.arm_request)
        self.assertFalse(self.task.servo_enabled)

    def finish_stow(self):
        self.assertEqual(self.task.state,'STOW')
        self.assertEqual(self.task.arm_request['cmd'],2)
        self.step(arm=True)
        if self.task.operation == 'STORAGE_PLACE':
            self.assertEqual(self.task.state,'RETURN_REFERENCE')
            self.xy = (self.task.reference[0]+.08,self.task.reference[1]-.04)
            self.assertLess(self.step()[0],0.)
            self.xy = self.task.reference
            self.step()
            self.step(dt=.15)
        self.assertEqual(self.task.state,'DONE')
        self.assertEqual(self.task.returned,self.task.operation == 'STORAGE_PLACE')
        self.assertEqual(self.task.final_alignment,'NOT_RUN')
        self.assertNotIn(6,self.actions)
        self.assertNotIn(7,self.actions)

    def test_first_visit_real_work_high_align_direct_switch_stow_and_offset(self):
        self.task = StorageDebugTask(self.p,'test',(2,1,3),0.,1,{})
        self.assertEqual(self.task.state,'WORK_ENTRY')
        self.step(arm=True)
        rings = {1:(20.,100.),2:(100.,100.),3:(180.,100.)}
        for _ in range(20):
            if self.task.state=='ALIGN_RING': break
            self.step(rings=rings)
        self.assertEqual(self.task.state,'ALIGN_RING')
        self.assertEqual(self.task.reference,self.xy)
        for ring in (2,1,3): self.align_item(ring=ring)
        self.assertEqual([(op,p) for op,_,_,p in self.task.confirmed],[('PLACE',2),('PLACE',1),('PLACE',3)])
        self.finish_stow()
        self.assertEqual(self.actions,[3,2])

    def test_second_visit_partial_scan_restores_once_then_switches_directly(self):
        self.task = StorageDebugTask(self.p,'test',(5,1,6),0.,2,{1:1,5:2,6:3})
        self.step(arm=True)
        for _ in range(3): self.step(materials=((5,(100.,90.)),))
        self.assertEqual(self.task.seen_colors,{5})
        self.assertGreater(self.task.arm_velocity[0],0.)
        for _ in range(3): self.step(materials=((1,(20.,90.)),(6,(180.,90.))))
        self.assertEqual(self.task.state,'SCAN_RESTORE_WORK')
        self.step(arm=True)
        for color in (5,1,6):
            self.assertEqual(self.task.velocity,(0.,0.,0.))
            self.align_item(color=color)
        self.assertEqual([(op,c,p) for op,_,c,p in self.task.confirmed],[('STACK',5,2),('STACK',1,1),('STACK',6,3)])
        self.finish_stow()
        self.assertEqual(self.actions,[3,3,2])

    def test_task_input_mapping_and_area_parameter_persistence(self):
        self.assertEqual(parse_task(dict(area='storage',visit=2,order='516',layout='156')),
                         ('storage',2,(5,1,6),{1:1,5:2,6:3}))
        for settings in [dict(area='invalid'),dict(visit=True),dict(visit=3),
                         dict(area='storage',visit=2,order='516',layout='123'),
                         dict(area='storage',visit=2,order='511',layout='156'),
                         dict(area='storage',visit=1,order='516')]:
            with self.assertRaises(ValueError):
                parse_task(settings)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'debug.yaml'
            copy_debug_config(path)
            values = {f['path']:get_value(self.config,f['path'],'storage') for f in area_fields('storage')}
            values['vision.target_uv_px'] = [123.,234.]
            saved = save_parameters(path,self.config,values,'storage')
            self.assertEqual(read_config(path),saved)
            self.assertEqual(saved['coarse'],self.config['coarse'])
            self.assertEqual(saved['raw'],self.config['raw'])
            self.assertEqual(saved['storage']['vision']['target_uv_px'],[123.,234.])
            previous = path.read_bytes()
            invalid = deepcopy(values)
            invalid['ring_search.scan_settle_frames'] = 0
            with self.assertRaises(ValueError):
                save_parameters(path,saved,invalid,'storage')
            self.assertEqual(path.read_bytes(),previous)


if __name__ == '__main__':
    unittest.main()
