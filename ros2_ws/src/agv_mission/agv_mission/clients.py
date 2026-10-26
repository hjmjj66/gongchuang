"""ROS clients shared by mock, integration and competition modes."""

from rclpy.action import ActionClient
from agv_interfaces.action import RunWorkBatch, ScanRoads
from agv_interfaces.srv import GetExecutorRuntime, PlanRoute, ResetExecutor, StopExecutor


class Clients:
    def __init__(self, node):
        prefixes = {'route': '/route', 'lidar': '/obstacles', 'work': '/vision'}
        self.get = {key: node.create_client(GetExecutorRuntime, path + '/get_runtime')
                    for key, path in prefixes.items()}
        self.stop = {key: node.create_client(StopExecutor, path + '/stop')
                     for key, path in prefixes.items()}
        self.reset = {key: node.create_client(ResetExecutor, path + '/reset_context')
                      for key, path in prefixes.items()}
        self.route = node.create_client(PlanRoute, '/route/plan')
        self.scan = ActionClient(node, ScanRoads, '/obstacles/scan')
        self.work = {area: ActionClient(node, RunWorkBatch, f'/vision/{area}/run_batch')
                     for area in ('raw', 'coarse', 'storage')}

    def ready(self):
        services = [*self.get.values(), *self.reset.values(), *self.stop.values(), self.route]
        return all(client.service_is_ready() for client in services) and self.scan.server_is_ready() and all(
            client.server_is_ready() for client in self.work.values())
