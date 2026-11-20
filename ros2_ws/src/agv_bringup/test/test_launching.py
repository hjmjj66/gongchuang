"""Verify launch supervision with harmless child processes, never hardware nodes."""

import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
import yaml

from agv_bringup.launching import CONFIGS, setup


def context_for(directory, use_lidar):
    context = LaunchContext()
    for name, (package, filename) in CONFIGS.items():
        context.launch_configurations[name + '_config'] = str(
            Path(get_package_share_directory(package)) / 'config' / filename)
    path = Path(directory) / 'mission.yaml'
    path.write_text(yaml.safe_dump({'mission_node': {'ros__parameters': {'use_lidar': use_lidar}}}))
    context.launch_configurations['mission_config'] = str(path)
    return context


class LaunchingTests(unittest.TestCase):
    def test_enabled_placeholder_fails_before_any_hardware_process(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('agv_bringup.launching.Node') as node, patch('agv_bringup.launching.preflight') as preflight:
                with self.assertRaisesRegex(Exception, '占位|launch file'):
                    setup(context_for(directory, True))
                node.assert_not_called()
                preflight.assert_not_called()

    def test_child_exit_stops_all_six_without_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            captures = []

            def fake_node(**kwargs):
                name = kwargs['name']
                captures.append(kwargs)
                marker = str(Path(directory) / (name + '.json'))
                code = (
                    'import json,os,signal,time; '
                    f'path={marker!r}; '
                    'signal.signal(signal.SIGINT, lambda *_: exit(0)); '
                    'open(path,"w").write(json.dumps(os.getpid())); '
                    f'time.sleep({0.5 if name == "screen_node" else 30})'
                )
                return ExecuteProcess(cmd=[sys.executable, '-c', code], output='log')

            with patch('agv_bringup.launching.Node', side_effect=fake_node), patch('agv_bringup.launching.preflight'):
                actions = setup(context_for(directory, False))
            qr = next(row for row in captures if row['name'] == 'qr_node')
            self.assertTrue(qr['parameters'][-1]['wait_for_mission'])
            mission = next(row for row in captures if row['name'] == 'mission_node')
            self.assertEqual(mission['parameters'][-1], {'mode': 'competition', 'screen_controlled': True})
            self.assertEqual(len(captures), 6)
            service = LaunchService(noninteractive=True)
            service.include_launch_description(LaunchDescription(actions))
            started = time.monotonic()
            service.run()
            self.assertLess(time.monotonic() - started, 10)
            markers = list(Path(directory).glob('*_node.json'))
            self.assertEqual(len(markers), 6)
            for path in markers:
                self.assertFalse(Path(f'/proc/{json.loads(path.read_text())}').exists())


if __name__ == '__main__':
    unittest.main()
