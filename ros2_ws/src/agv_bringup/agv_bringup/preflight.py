"""Read-only discovery; refuse competing controllers before opening any devices."""

import time

import rclpy
from rclpy.node import NodeNameNonExistentError


def preflight():
    rclpy.init()
    node = rclpy.create_node('competition_preflight')
    conflicts = set()
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for topic in ('/C/controll', '/C/odometry', '/C/status'):
            if node.get_publishers_info_by_topic(topic):
                conflicts.add(topic)
        owners = {'mission_node', 'screen_node', 'route_planner_node', 'cboard_bridge_node',
                  'coarse_node', 'qr_node', 'route_debug_controller', 'coarse_debug_controller',
                  'ring_debug_node', 'material_debug_node'}
        for name, namespace in node.get_node_names_and_namespaces():
            owner = namespace.rstrip('/') + '/' + name
            if name in owners:
                conflicts.add(owner)
            try:
                services = node.get_service_names_and_types_by_node(name, namespace)
            except NodeNameNonExistentError:
                continue
            for service, _ in services:
                if service.startswith(('/mission/', '/route/', '/vision/', '/obstacles/',
                                       '/tools/route_debug/')) or service == '/C/arm_command':
                    conflicts.add(f'{service} (服务端：{owner})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if conflicts:
        raise RuntimeError('请先退出已有比赛、调试或设备节点：' + ', '.join(sorted(conflicts)))
