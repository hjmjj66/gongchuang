"""Exercise persisted tuning through the real ROS parameter API."""

from pathlib import Path
import tempfile
import unittest

import rclpy
from rclpy.parameter import Parameter
import yaml

from agv_planning.motion_config import MOTION_PARAMETERS, save_motion_parameters
from agv_planning.route_planner_node import RoutePlannerNode


SOURCE = Path(__file__).resolve().parents[1] / 'config/planner.yaml'


class MotionConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'planner.yaml'
        self.original = SOURCE.read_text()
        self.path.write_text(self.original)
        self.values = yaml.safe_load(self.original)['route_planner_node']['ros__parameters']
        self.node = self.make_node()
        self.addCleanup(lambda: self.node.destroy_node())

    def make_node(self):
        return RoutePlannerNode(parameter_overrides=[
            Parameter('debug_motion_config_path', value=str(self.path)),
            *[Parameter(name, value=float(self.values[name])) for name in MOTION_PARAMETERS]])

    def set_values(self, **values):
        return self.node.set_parameters_atomically([
            Parameter(name, value=value) for name, value in values.items()])

    def test_save_changes_real_output_preserves_notes_and_survives_restart(self):
        result = self.set_values(translation_pid_kp=3., translation_max_vx_mps=.02,
                                 translation_max_vy_mps=.015, angle_pid_kp=1.5,
                                 translation_arrival_tolerance_mm=5., turn_arrival_tolerance_deg=.5)
        self.assertTrue(result.successful, result.reason)
        self.assertEqual(self.node.get_parameter('translation_pid_kp').value, 3.)
        self.node._segment_baseline = (0., 0., 0.)
        self.node._position = (0., 0.)
        self.node._yaw_deg = 0.
        self.node._target_position = (.01, -.01)
        velocity, _, done = self.node._translation_motion(.02)
        self.assertEqual(velocity, (.02, -.015))
        self.assertFalse(done)
        self.assertEqual(self.node._angle_pid.update(1., 0., .02), 1.5)
        self.assertEqual(self.node._angle_pid.update(.4, 0., .02), 0.)
        saved = self.path.read_text()
        self.assertEqual([line.split('#', 1)[1] for line in saved.splitlines() if '#' in line],
                         [line.split('#', 1)[1] for line in self.original.splitlines() if '#' in line])
        self.values = yaml.safe_load(saved)['route_planner_node']['ros__parameters']
        self.assertEqual(self.values['work_heading'], yaml.safe_load(self.original)['route_planner_node']['ros__parameters']['work_heading'])
        self.node.destroy_node()
        self.node = self.make_node()
        self.assertEqual(self.node._position_pid.kp, 3.)
        self.assertEqual(self.node._position_pid.tolerance, .005)
        self.assertEqual(self.node._angle_pid.tolerance, .5)

    def test_running_planner_rejects_save_without_changing_file_or_pid(self):
        before = self.node._position_pid
        self.node._active = True
        result = self.set_values(translation_pid_kp=4.)
        self.assertFalse(result.successful)
        self.assertIn('先停止', result.reason)
        self.assertIs(self.node._position_pid, before)
        self.assertEqual(self.path.read_text(), self.original)

    def test_invalid_batch_never_partially_applies(self):
        for name, value in [('angle_pid_kp', 0.), ('angle_pid_ki', -1.),
                            ('translation_max_vx_mps', float('nan')),
                            ('angle_max_speed_deg_s', float('inf')),
                            ('translation_arrival_tolerance_mm', 1200.),
                            ('turn_arrival_tolerance_deg', 90.)]:
            with self.subTest(name=name, value=value):
                result = self.set_values(translation_pid_kp=9., **{name: value})
                self.assertFalse(result.successful)
                self.assertEqual(self.node._position_pid.kp, self.values['translation_pid_kp'])
                self.assertEqual(self.node.get_parameter('translation_pid_kp').value,
                                 self.values['translation_pid_kp'])
                self.assertEqual(self.path.read_text(), self.original)

    def test_failed_write_keeps_runtime_parameters(self):
        self.node._motion_config_path = str(self.path.parent / 'missing.yaml')
        result = self.set_values(angle_pid_kp=3.)
        self.assertFalse(result.successful)
        self.assertEqual(self.node._angle_pid.kp, self.values['angle_pid_kp'])
        self.assertEqual(self.node.get_parameter('angle_pid_kp').value, self.values['angle_pid_kp'])
        self.assertEqual(self.path.read_text(), self.original)

    def test_normal_planner_parameters_remain_read_only(self):
        node = RoutePlannerNode()
        try:
            result = node.set_parameters_atomically([Parameter('angle_pid_kp', value=3.)])
            self.assertFalse(result.successful)
            self.assertTrue(node.describe_parameter('angle_pid_kp').read_only)
        finally:
            node.destroy_node()

    def test_save_adds_missing_fields_and_handles_flow_yaml(self):
        values = {name: default for name, (_, default) in MOTION_PARAMETERS.items()}
        for text in ('route_planner_node:\n  ros__parameters:\n    default_edge_length_mm: 850 # keep\n',
                     'route_planner_node: {ros__parameters: {default_edge_length_mm: 850}}\n'):
            with self.subTest(text=text):
                self.path.write_text(text)
                save_motion_parameters(self.path, values)
                saved = yaml.safe_load(self.path.read_text())['route_planner_node']['ros__parameters']
                self.assertEqual(saved, dict(default_edge_length_mm=850, **values))
                if '# keep' in text:
                    self.assertIn('# keep', self.path.read_text())
