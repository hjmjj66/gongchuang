"""Web raw reuses the production core; only PICK is replaced with a one-second wait."""
from copy import deepcopy
from motion_fixture import copy_debug_config
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from agv_perception.work_control import GantryPose, Observation, Pose
from coarse_debug.config import area_fields, get_value, parse_task, read_config, save_parameters
from coarse_debug.task import RawDebugTask

CONFIG = Path(__file__).parents[1]/'config/debug.yaml'


class RawDebugTests(unittest.TestCase):
    def setUp(self):
        self.config = read_config(CONFIG)
        self.p = self.config['raw']
        self.p['vision']['target_uv_px'] = [100., 100.]
        self.now, self.sequence = 0., 0
        self.xy = (1.,2.)
        self.task = RawDebugTask(self.p,'raw',(2,1,3),0.)
        self.assertEqual(self.task.state,'ENTER_WORK_POSE')
        self.assertEqual(self.task.arm_request['cmd'],3)
        self.step(arm=True)
        self.step()
        self.step(dt=.11)
        self.xy=self.task.approach_target
        self.step()
        self.step(dt=.11)
        self.assertEqual(self.task.state,'WAIT_REQUIRED')

    def step(self,points=None,dt=.05,arm=False):
        self.now += dt
        self.sequence += 1
        obs = None if points is None else Observation('material',self.sequence,self.now,(201,201),{},tuple(points.items()))
        status = SimpleNamespace(command_id=self.task.arm_request['command_id'],state='COMPLETED') if arm else None
        return self.task.tick(self.now,Pose(*self.xy,0.,self.now),obs,status,GantryPose(120,.3,self.now))

    def test_real_work_and_stow_each_mock_pick_one_second_then_actual_restore(self):
        for color in (2,1,3):
            for _ in range(self.p['grasp_gate']['qualified_frames']):
                self.step({2:(100.,100.),1:(100.,100.),3:(100.,100.)})
            self.assertEqual(self.task.state,'PICK')
            self.assertEqual(self.task.tracked_color,color)
            self.assertIsNone(self.task.arm_request)
            self.assertEqual(self.task.arm_velocity,(0.,0.))
            self.step(dt=.99)
            self.assertEqual(self.task.state,'PICK')
            self.step(dt=.02)
            self.assertEqual(self.task.state,'RESTORE_WORK')
            self.assertEqual(self.task.arm_request['cmd'],3)
            self.step(arm=True)
        self.assertEqual(self.task.state,'STOW_ARM')
        self.assertEqual(self.task.arm_request['cmd'],2)
        self.assertFalse(self.task.returned)
        self.step(arm=True)
        self.assertEqual(self.task.state,'RETURN_REFERENCE')
        self.xy=self.task.reference
        self.step()
        self.step(dt=.11)
        self.assertEqual(self.task.state,'DONE')
        self.assertEqual([x[2] for x in self.task.confirmed],[2,1,3])

    def test_live_gantry_and_loss_hold_are_not_mocked(self):
        self.step({2:(70.,70.)})
        self.assertGreater(self.task.arm_velocity[0],0.)
        self.assertGreater(self.task.arm_velocity[1],0.)
        self.assertEqual(self.task.velocity,(0.,0.,0.))
        self.step({})
        self.step({},dt=.9)
        self.assertEqual(self.task.state,'WAIT_REQUIRED')
        self.step({},dt=1000.)
        self.assertEqual(self.task.state,'WAIT_REQUIRED')
        self.assertIsNone(self.task.arm_request)

    def test_color_input_and_independent_saved_parameters(self):
        self.assertEqual(parse_task(dict(area='raw',order='516',visit=2)),('raw',1,(5,1,6),{}))
        for order in ('112','1234','789','',213):
            with self.assertRaises(ValueError):
                parse_task(dict(area='raw',order=order))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'debug.yaml'
            copy_debug_config(path)
            original=read_config(path)
            values={f['path']:get_value(original,f['path'],'raw') for f in area_fields('raw')}
            values['gantry_servo.max_fb_rpm']=1.5
            values['motion_prediction.enabled']=False
            values['motion_prediction.extra_delay_s']=0.
            saved=save_parameters(path,original,values,'raw')
            self.assertEqual(read_config(path),saved)
            self.assertFalse(saved['raw']['motion_prediction']['enabled'])
            for area in ('coarse','storage'):
                self.assertEqual(saved[area],original[area])
            previous=path.read_bytes()
            invalid=deepcopy(values)
            invalid['grasp_gate.small_fb_rpm']=0
            with self.assertRaises(ValueError):
                save_parameters(path,saved,invalid,'raw')
            self.assertEqual(path.read_bytes(),previous)
            for name,value in (('motion_prediction.enabled',1),('motion_prediction.max_horizon_s',.6),
                               ('motion_prediction.extra_delay_s',.2)):
                invalid=deepcopy(values)
                invalid[name]=value
                with self.subTest(name=name), self.assertRaises(ValueError):
                    save_parameters(path,saved,invalid,'raw')
                self.assertEqual(path.read_bytes(),previous)
            invalid=deepcopy(values)
            invalid['arm.pick_cmd']=9
            with self.assertRaises(ValueError):
                save_parameters(path,saved,invalid,'raw')
            self.assertEqual(path.read_bytes(),previous)


if __name__=='__main__':
    unittest.main()
