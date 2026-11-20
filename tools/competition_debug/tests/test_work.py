"""Run every task made by the real host through its full controller states."""

from pathlib import Path
from types import SimpleNamespace
import unittest

from agv_perception.work_control import Observation, Pose, GantryPose
from competition_debug.work import make_task, read_configs, BatchWork

ROOT = Path(__file__).resolve().parents[3]


class TaskTests(unittest.TestCase):
    def test_full_tasks_keep_region_specific_returns_and_stow(self):
        config=read_configs({a:ROOT/'ros2_ws/src/agv_perception/config'/(a+'.yaml') for a in ('raw','coarse','storage')})
        for p in config.values():
            p['vision']['target_uv_px']=[100.,100.]
            p['vision']['stack_target_uv_px']=[100.,100.]
            p.pop('gantry_presets',None)
        cases=(('raw','RAW_PICK',()),('coarse','COARSE_PROCESS',(2,3,1)),
               ('storage','STORAGE_PLACE',(2,3,1)),('storage','STORAGE_STACK',(3,2,1)))
        for execution in ('real','simulated'):
            for area,operation,positions in cases:
                with self.subTest(execution=execution,operation=operation):
                    colors=(1,5,6) if operation!='STORAGE_STACK' else (5,1,6)
                    mapping={1:2,5:3,6:1}
                    task=make_task(config[area],area,operation,colors,positions,mapping,execution,'test',0.)
                    now=x=y=0.
                    actions=[];states=[];dwells={}
                    arm=SimpleNamespace(command_id='',state='COMPLETED',idle=True,pending=False)
                    for sequence in range(1,2001):
                        now+=.05
                        x+=task.velocity[0]*.05;y+=task.velocity[1]*.05
                        if task.arm_request:
                            request=task.arm_request
                            if arm.command_id!=request['command_id']:
                                actions.append(request['cmd'])
                                arm.command_id=request['command_id']
                        kind=task.vision_kind or 'ring'
                        rings={1:(20.,100.),2:(100.,100.),3:(180.,100.)}
                        if task.state=='ALIGN_RING':rings[task.positions[task.item]]=(100.,100.)
                        materials=tuple((c,(100.,100.)) for c in colors)
                        obs=Observation(kind,sequence,now,(201,201),rings,materials,geometry_target=task.vision_target)
                        pose=Pose(x,y,0.,now,0.,0.)
                        if task.state not in states:states.append(task.state)
                        if execution=='simulated' and task.state in ('PICK','PLACE','STACK'):
                            self.assertIsNone(task.arm_request)
                            self.assertEqual(task.arm_velocity,(0.,0.))
                            self.assertEqual(task.velocity,(0.,0.,0.))
                            key=(task.state,task.item)
                            dwells[key]=dwells.get(key,0)+.05
                        task.tick(now,pose,obs,arm,GantryPose(0,0.,now))
                        if task.finished:break
                    self.assertEqual(task.state,'DONE',(states,task.reason,task.diagnostic))
                    self.assertEqual([r[2] for r in task.confirmed[:3]],list(colors))
                    self.assertEqual(len(task.confirmed),6 if area=='coarse' else 3)
                    self.assertEqual(actions[-1],2)
                    if execution=='simulated':
                        self.assertFalse(set(actions)&{4,5,6,7,8},actions)
                        self.assertEqual(len(dwells),len(task.confirmed))
                        self.assertTrue(all(s>=.99 for s in dwells.values()))
                    else:
                        expected={'RAW_PICK':4,'COARSE_PROCESS':6,'STORAGE_PLACE':6,'STORAGE_STACK':7}[operation]
                        self.assertEqual(actions.count(expected),3)
                        if area=='coarse':self.assertEqual(actions.count(5),3)
                    if area=='raw':
                        self.assertIn('APPROACH',states)
                        self.assertIn('RETURN_REFERENCE',states)
                        self.assertTrue(task.returned)
                        self.assertAlmostEqual(x,0.,delta=.001)
                        self.assertAlmostEqual(y,0.,delta=.001)
                    elif operation!='STORAGE_STACK':
                        self.assertIn('RETURN_REFERENCE',states)
                        self.assertTrue(task.returned)
                        self.assertAlmostEqual(x,task.reference[0],delta=.001)
                        self.assertAlmostEqual(y,task.reference[1],delta=.001)
                    else:
                        self.assertNotIn('RETURN_REFERENCE',states)
                        self.assertIn('SCAN_FIRST_LAYER',states)

    def test_host_results_identify_dwell_and_real_and_do_not_repeat_events(self):
        host=BatchWork.__new__(BatchWork)
        host.seen=0;host.last_state=None;host.last_reason=''
        task=SimpleNamespace(execution='simulated',confirmed=[('PICK',1,6,0)],state='DONE',
            velocity=(0.,0.,0.),diagnostic={},reason='',entry_alignment='NOT_RUN')
        result=host.status(task)
        self.assertEqual(result['results'][0]['outcome'],'DWELL_COMPLETED')
        self.assertIn('停留1秒',result['events'][0])
        self.assertEqual(host.status(task)['events'],[])
        host.seen=0;task.execution='real'
        self.assertEqual(host.status(task)['results'][0]['outcome'],'REAL_COMPLETED')
