"""Shared run binding, compare-and-set reset and process identity contract."""

import time
import uuid
import math

from std_msgs.msg import String
from agv_interfaces.msg import ExecutorRuntime
from agv_interfaces.srv import GetExecutorRuntime, ResetExecutor, StopExecutor


class ExecutorContract:
    def __init__(self, node, prefix, source, lease_timeout, runtime, stop, reset,
                 debug_allow_any_pose=False):
        if source not in ('REAL', 'MOCK') or not math.isfinite(lease_timeout) or lease_timeout <= 0:
            raise ValueError('source must be REAL/MOCK and lease_timeout must be positive')
        self.node, self.source = node, source
        self.session_id = str(uuid.uuid4())
        self.run_id = ''
        self.last_lease = None
        self.lease_timeout = lease_timeout
        self._resets = {}
        self._runtime, self._stop, self._reset = runtime, stop, reset
        self._debug_allow_any_pose = debug_allow_any_pose
        self._services = [
            node.create_service(GetExecutorRuntime, prefix + '/get_runtime', self.get),
            node.create_service(StopExecutor, prefix + '/stop', self.stop),
            node.create_service(ResetExecutor, prefix + '/reset_context', self.reset),
        ]
        self._sub = node.create_subscription(String, '/mission/heartbeat', self._heartbeat, 1)
        self._timer = node.create_timer(min(0.2, lease_timeout / 3), self._watchdog)

    def snapshot(self):
        result = self._runtime()
        result.run_id = self.run_id
        result.executor_session_id = self.session_id
        result.source = self.source
        return result

    def authorized(self, run_id):
        return run_id == self.run_id and (not run_id or self.lease_valid())

    def lease_valid(self):
        return self.last_lease is not None and time.monotonic() - self.last_lease < self.lease_timeout

    def _heartbeat(self, msg):
        if self.run_id and msg.data == self.run_id:
            self.last_lease = time.monotonic()

    def _watchdog(self):
        state = self.snapshot()
        if self.run_id and state.busy and not self.lease_valid():
            self._stop('调度心跳超时')

    def get(self, request, response):
        response.runtime = self.snapshot()
        return response

    def stop(self, request, response):
        state = self.snapshot()
        if request.run_id != self.run_id:
            response.reason = 'run_id mismatch'
        elif request.task_id != state.active_task_id and state.busy:
            response.reason = 'active task_id mismatch'
        elif not state.busy and request.task_id and request.task_id != state.last_task_id:
            response.reason = 'task_id is neither active nor the last task'
        else:
            self._stop('显式停止')
            response.accepted = True
        response.runtime = self.snapshot()
        return response

    def reset(self, request, response):
        identity = (request.expected_run_id, request.run_id, request.home_node, request.initial_heading)
        state = self.snapshot()
        previous = self._resets.get(request.reset_id)
        if previous == identity and request.run_id == self.run_id:
            response.accepted = True
        elif previous is not None:
            response.reason = 'stale or conflicting reset_id'
        elif not request.run_id or not request.reset_id or request.expected_run_id != self.run_id:
            response.reason = 'reset compare-and-set failed'
        elif request.run_id == self.run_id or state.busy or not state.quiescent or state.arm_pending:
            response.reason = 'reset requires a new run and a quiescent executor'
        elif not self._valid_initial_pose(request.home_node, request.initial_heading):
            response.reason = 'invalid home node or initial heading'
        else:
            self._reset(request)
            self.run_id = request.run_id
            self.last_lease = time.monotonic()
            self._resets[request.reset_id] = identity
            response.accepted = True
        response.runtime = self.snapshot()
        return response

    def _valid_initial_pose(self, node, heading):
        if self._debug_allow_any_pose:
            return node in range(1, 10) and heading in ('UP', 'RIGHT', 'DOWN', 'LEFT')
        return node in (1, 3) and heading == ('DOWN' if node == 1 else 'UP')
