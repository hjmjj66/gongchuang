"""Inspect real launch actions without starting any node or hardware."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

ROOT=Path(__file__).resolve().parents[3]


class LaunchTests(unittest.TestCase):
    def load(self):
        spec=importlib.util.spec_from_file_location('full_debug_launch',ROOT/'tools/competition_debug/launch/competition_debug.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        context=LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action,DeclareLaunchArgument):action.execute(context)
        return module,context

    def test_default_port_sources_and_single_controller(self):
        module,context=self.load()
        self.assertEqual(context.launch_configurations['port'],'8087')
        self.assertEqual(context.launch_configurations['raw_config'],str(ROOT/'ros2_ws/src/agv_perception/config/raw.yaml'))
        actions=module.setup(context)
        self.assertEqual(len(actions),6)  # Three supervised processes and three exit handlers.
        controller=actions[-1]
        command=' '.join(perform_substitutions(context,v) for v in controller.cmd)
        self.assertIn('competition_debug.node',command)
        self.assertIn('--raw-config',command)
        self.assertIn('8087',command)
        self.assertNotIn('mission_node',command)

    def test_enabled_placeholder_lidar_fails_before_hardware_actions(self):
        module,context=self.load()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'mission.yaml'
            path.write_text(yaml.safe_dump({'mission_node':{'ros__parameters':{'use_lidar':True}}}))
            context.launch_configurations['mission_config']=str(path)
            with self.assertRaisesRegex(RuntimeError,'占位'):
                module.setup(context)
