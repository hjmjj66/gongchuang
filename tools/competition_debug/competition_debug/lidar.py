"""Existing ScanRoads / runtime protocol, including reset and stop acknowledgement."""

import time
import uuid

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_msgs.msg import String
from agv_interfaces.action import ScanRoads
from agv_interfaces.srv import GetExecutorRuntime, ResetExecutor, StopExecutor
from agv_planning.planner import Topology


class Lidar:
    def __init__(self, node, timeout):
        self.node, self.timeout = node, timeout
        self.clients = {name: node.create_client(kind, '/obstacles/'+name) for name, kind in
                        (('get_runtime', GetExecutorRuntime), ('reset_context', ResetExecutor), ('stop', StopExecutor))}
        self.action = ActionClient(node, ScanRoads, '/obstacles/scan')
        self.lease = node.create_publisher(String, '/mission/heartbeat', 1)
        self.runtime = None
        self.at = 0.
        self.pending = None
        self.error = ''
        self.run_id = ''
        self.session_id = ''
        self.task_id = ''
        self.result = None
        self.handle = None
        self.stopping = False
        self.stop_sent = False
        self.reset_run_id = ''
        self.timer = node.create_timer(.2, self.query)

    def ready(self):
        return all(c.service_is_ready() for c in self.clients.values()) and self.action.server_is_ready()

    def query(self):
        if self.run_id:
            self.lease.publish(String(data=self.run_id))
        if self.pending:
            if time.monotonic()-self.pending[1] > self.timeout:
                self.error = '雷达接口响应超时，结果未知'
            return
        if self.clients['get_runtime'].service_is_ready():
            self.call('get_runtime', GetExecutorRuntime.Request(), self.update)

    def call(self, kind, request, callback):
        future = self.clients[kind].call_async(request)
        self.pending = (future, time.monotonic())

        def done(f):
            self.pending = None
            try:
                callback(f.result())
            except Exception as exc:
                self.error = str(exc)
        future.add_done_callback(done)

    def update(self, response):
        rt = response.runtime
        if rt.source != 'REAL':
            raise RuntimeError('雷达拒绝非真实执行端')
        if self.session_id and rt.executor_session_id != self.session_id:
            raise RuntimeError('雷达执行端进程发生变化')
        self.runtime, self.at = rt, time.monotonic()
        if self.stopping and rt.busy and not self.stop_sent and self.pending is None:
            self.stop_sent = True
            self.call('stop', StopExecutor.Request(run_id=rt.run_id, task_id=rt.active_task_id), self.stop_result)

    def prepare(self, run_id, home, heading):
        if self.error:
            raise RuntimeError(self.error)
        rt = self.runtime
        if rt is None or time.monotonic()-self.at > self.timeout:
            return False
        if rt.run_id == run_id and rt.config_ready and rt.travel_ready and rt.quiescent and rt.motion_released and not rt.busy:
            return True
        if self.pending is None and self.reset_run_id != run_id:
            if rt.busy or not rt.quiescent or not rt.motion_released:
                raise RuntimeError('雷达执行端尚未释放')
            self.run_id, self.session_id = run_id, rt.executor_session_id
            self.reset_run_id = run_id
            self.stopping = self.stop_sent = False
            self.task_id = ''
            self.result = self.handle = None
            self.call('reset_context', ResetExecutor.Request(expected_run_id=rt.run_id, run_id=run_id,
                reset_id=uuid.uuid4().hex, home_node=home, initial_heading=heading), self.reset_result)
        return False

    def reset_result(self, response):
        if not response.accepted:
            raise RuntimeError('雷达复位被拒绝：'+response.reason)
        self.update(response)

    def start(self, run_id):
        self.task_id = uuid.uuid4().hex
        self.run_id = run_id
        self.result = None
        self.goal_future = self.action.send_goal_async(ScanRoads.Goal(run_id=run_id, task_id=self.task_id))
        self.goal_at = time.monotonic()
        self.goal_future.add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                raise RuntimeError('雷达扫描任务被拒绝')
            if self.stopping:
                self.handle.cancel_goal_async()
            self.handle.get_result_async().add_done_callback(self.completed)
        except Exception as exc:
            self.error = str(exc)

    def completed(self, future):
        try:
            response = future.result()
            r = response.result
            if (r.run_id != self.run_id or r.task_id != self.task_id or r.source != 'REAL' or
                    r.executor_session_id != self.session_id):
                raise RuntimeError('雷达扫描结果身份冲突')
            if not self.stopping and (response.status != GoalStatus.STATUS_SUCCEEDED or
                    r.outcome != 'COMPLETED' or not r.travel_ready or not r.map_revision):
                raise RuntimeError('真实雷达扫描未完整完成：'+r.reason)
            self.result = sorted(Topology().normalize_blocked((e.a, e.b) for e in r.blocked_edges))
        except Exception as exc:
            self.error = str(exc)

    def tick(self):
        if self.error:
            raise RuntimeError(self.error)
        if time.monotonic()-self.at > self.timeout:
            raise RuntimeError('雷达运行状态过期')
        if self.handle is None and time.monotonic()-self.goal_at > self.timeout:
            raise RuntimeError('雷达任务受理超时，结果未知')
        return dict(done=self.result is not None, blocked_edges=self.result, label='等待真实雷达扫描',
                    velocity=[0., 0., 0.])

    def stop(self):
        self.stopping = True
        if self.handle:
            self.handle.cancel_goal_async()

    def stop_result(self, response):
        if not response.accepted:
            self.error = '雷达停止被拒绝：'+response.reason
        self.update(response)

    def released(self):
        rt = self.runtime
        goal_pending = self.task_id and (self.handle is None or self.result is None)
        return bool(not self.error and not self.pending and not goal_pending and rt and
                    time.monotonic()-self.at <= self.timeout and rt.quiescent and rt.motion_released and
                    not rt.busy and not rt.arm_pending)
