"""Displayed-frame capture, scoped persistence and independent formal storage aims."""

from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

from agv_perception.storage_control import StorageTask
from agv_perception.work_control import Observation, Pose
from coarse_debug.calibration import CalibrationFrames, save_calibration
from coarse_debug.config import read_config
from coarse_debug.tracking import TrackingTask


ROOT = Path(__file__).resolve().parents[3]
DEBUG = Path(__file__).resolve().parents[1]/'config/debug.yaml'
PRODUCTION = ROOT/'ros2_ws/src/agv_perception/config'


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='calibration_test_')
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.debug = self.folder/'debug.yaml'
        self.debug.write_text(DEBUG.read_text())
        self.config = read_config(self.debug)
        self.paths = {area:self.folder/f'{area}.yaml' for area in ('raw','coarse','storage')}
        for area,path in self.paths.items():
            path.write_text((PRODUCTION/f'{area}.yaml').read_text())

    def test_three_points_sync_to_only_their_intended_formal_and_web_fields(self):
        for category,point in (('raw',[321.25,240.5]),('ring',[111.5,222.25]),('stack',[444.25,333.5])):
            self.config = save_calibration(self.debug,self.config,category,point,self.paths)
        self.assertEqual(self.config['raw']['vision']['target_uv_px'],[321.25,240.5])
        for area in ('coarse','storage'):
            self.assertEqual(self.config[area]['vision']['target_uv_px'],[111.5,222.25])
        self.assertEqual(self.config['storage']['vision']['stack_target_uv_px'],[444.25,333.5])
        for area,path in self.paths.items():
            old = yaml.safe_load((PRODUCTION/f'{area}.yaml').read_text())
            new = yaml.safe_load(path.read_text())
            self.assertEqual(new[area]['vision']['target_uv_px'],self.config[area]['vision']['target_uv_px'])
            if area == 'storage':
                self.assertEqual(new[area]['vision']['stack_target_uv_px'],[444.25,333.5])
                old[area]['vision']['stack_target_uv_px']=[444.25,333.5]
            old[area]['vision']['target_uv_px']=self.config[area]['vision']['target_uv_px']
            self.assertEqual(new,old)
            self.assertEqual(path.read_text().splitlines()[0],(PRODUCTION/f'{area}.yaml').read_text().splitlines()[0])
            self.assertEqual(path.read_text().count('null'),(PRODUCTION/f'{area}.yaml').read_text().count('null'))

    def test_partial_replace_failure_rolls_back_all_files(self):
        import os
        originals={path:path.read_bytes() for path in (self.debug,*self.paths.values())}
        replace=os.replace
        count=0
        def fail_second(source,destination):
            nonlocal count
            count+=1
            if count == 2:
                raise OSError('模拟写入失败')
            replace(source,destination)
        with patch('coarse_debug.persistence.os.replace',side_effect=fail_second):
            with self.assertRaises(OSError):
                save_calibration(self.debug,self.config,'ring',[30.,40.],self.paths)
        for path,data in originals.items():
            self.assertEqual(path.read_bytes(),data)
        self.assertFalse(list(self.folder.glob('*.calibration.*')))

    def test_source_and_installed_copies_receive_the_same_point_without_copying_other_parameters(self):
        installed=self.folder/'installed-storage.yaml'
        content=self.paths['storage'].read_text().replace('    work_pose_cmd: 3','    work_pose_cmd: 2')
        installed.write_text(content)
        paths=dict(self.paths,storage=[self.paths['storage'],installed])
        save_calibration(self.debug,self.config,'ring',[23.,45.],paths)
        for path in (self.paths['storage'],installed):
            value=yaml.safe_load(path.read_text())['storage']
            self.assertEqual(value['vision']['target_uv_px'],[23.,45.])
        self.assertEqual(yaml.safe_load(installed.read_text())['storage']['arm']['work_pose_cmd'],2)

    def test_legacy_storage_keeps_its_second_visit_point_when_ring_is_calibrated(self):
        for path in (self.debug,self.paths['storage']):
            document=yaml.safe_load(path.read_text())
            document['storage']['vision'].pop('stack_target_uv_px', None)
            document['storage']['vision']['target_uv_px']=[]
            path.write_text(yaml.safe_dump(document,allow_unicode=True))
        self.config=read_config(self.debug)
        result=save_calibration(self.debug,self.config,'ring',[10.,20.],self.paths)
        self.assertEqual(result['storage']['vision']['stack_target_uv_px'],[])
        self.assertEqual(read_config(self.debug)['storage']['vision']['stack_target_uv_px'],[])
        formal=yaml.safe_load(self.paths['storage'].read_text())
        self.assertEqual(formal['storage']['vision']['stack_target_uv_px'],[])
        self.assertEqual(formal['storage']['vision']['target_uv_px'],[10.,20.])

    def test_missing_or_malformed_formal_config_cannot_partially_save(self):
        for bad in ('storage:\n  vision: [\n', 'wrong_root: {}\n'):
            original=self.debug.read_bytes()
            self.paths['coarse'].write_text(bad)
            with self.assertRaises(ValueError):
                save_calibration(self.debug,self.config,'ring',[10.,20.],self.paths)
            self.assertEqual(self.debug.read_bytes(),original)

    def test_capture_is_from_the_displayed_packet_and_rejects_wrong_stale_or_ambiguous_targets(self):
        frames=CalibrationFrames()
        obs=Observation('material',4,1.,(1280,720),{},((5,(412.25,333.5)),))
        header=frames.register(obs,'raw','selection1')
        settings=dict(frame_token=header['token'],target=5)
        newer=Observation('material',5,1.1,(1280,720),{},((5,(900.,600.)),))
        frames.register(newer,'raw','selection1')
        sample=frames.capture(settings,'raw','selection1',1.2,.5)
        self.assertEqual(sample['point'],[412.25,333.5])
        self.assertEqual(sample['sequence'],4)
        for category,selection,now,target in (('stack','selection1',1.2,5),('raw','selection2',1.2,5),
                                            ('raw','selection1',2.,5),('raw','selection1',1.2,6)):
            with self.assertRaises(ValueError):
                frames.capture(dict(settings,target=target),category,selection,now,.5)
        duplicate=Observation('material',6,1.1,(1280,720),{},((5,(400.,300.)),(5,(500.,300.))))
        header=frames.register(duplicate,'raw','selection1')
        with self.assertRaisesRegex(ValueError,'多个同色'):
            frames.capture(dict(frame_token=header['token'],target=5),'raw','selection1',1.2,.5)

    def test_formal_storage_and_web_tracking_use_different_ring_and_stack_aims(self):
        p=deepcopy(self.config['storage'])
        p.pop('gantry_presets', None)  # Exercise visual aims independently of saved prepositions.
        p['vision'].update(target_uv_px=[80.,90.],stack_target_uv_px=[120.,130.])
        for operation,point,kind in (('STORAGE_PLACE',(80.,90.),'ring'),('STORAGE_STACK',(120.,130.),'material')):
            task=StorageTask(p,'formal',(1,2,3),(1,2,3),0.,operation,{1:1,2:2,3:3})
            task._enter('ALIGN_RING' if kind == 'ring' else 'ALIGN_MATERIAL',0.)
            obs=Observation(kind,1,.1,(201,201),{1:point},((1,point),))
            task.tick(.1,Pose(0.,0.,0.,.1),obs,SimpleNamespace(idle=True,pending=False))
            self.assertEqual(task.diagnostic['target_uv'],list(point))
            self.assertEqual(task.arm_velocity,(0.,0.))
            web=TrackingTask(p,'web',1,kind,0.,'storage',1 if kind == 'ring' else 2)
            web.tick(.1,observation=obs)
            self.assertEqual(web.arm_velocity,(0.,0.))
            self.assertEqual(web.state,'TRACK_HOLD')


if __name__ == '__main__':
    unittest.main()
