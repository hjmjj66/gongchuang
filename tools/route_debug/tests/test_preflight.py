"""Read-only ROS graph regression tests; use an isolated ROS domain."""

import os
from pathlib import Path
import subprocess
import unittest


@unittest.skipUnless(os.environ.get('AGV_ROUTE_DEBUG_ROS_TEST') == '1',
                     'set AGV_ROUTE_DEBUG_ROS_TEST=1 in an isolated ROS domain')
class PreflightTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('preflight_test_peer')
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(self.node.destroy_node)

    def check(self):
        tool = Path(__file__).resolve().parents[1]
        env = dict(os.environ, PYTHONPATH=str(tool) + os.pathsep + os.environ.get('PYTHONPATH', ''))
        return subprocess.run(['/usr/bin/python3', '-m', 'route_debug.node', '--check'],
                              env=env, capture_output=True, text=True, timeout=15)

    def test_empty_graph_is_allowed(self):
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_clients_without_servers_are_allowed(self):
        from std_srvs.srv import Trigger
        self.node.create_client(Trigger, '/C/reset_fault')
        self.node.create_client(Trigger, '/tools/route_debug/stop')
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_reset_server_with_custom_node_name_is_rejected(self):
        from std_srvs.srv import Trigger
        self.node.create_service(Trigger, '/C/reset_fault', lambda req, res: res)
        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('/C/reset_fault (服务端：/preflight_test_peer)', result.stderr)

    def test_private_server_is_rejected(self):
        from std_srvs.srv import Trigger
        self.node.create_service(Trigger, '/tools/route_debug/stop', lambda req, res: res)
        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('/tools/route_debug/stop (服务端：/preflight_test_peer)', result.stderr)

    def test_control_publisher_is_rejected(self):
        from agv_interfaces.msg import ControlCommand
        self.node.create_publisher(ControlCommand, '/C/controll', 1)
        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('/C/controll', result.stderr)

    def test_known_control_node_is_rejected(self):
        peer = self.rclpy.create_node('mission_node')
        self.addCleanup(peer.destroy_node)
        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('/mission_node', result.stderr)


if __name__ == '__main__':
    unittest.main()
