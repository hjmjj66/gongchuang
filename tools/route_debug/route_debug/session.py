"""Serial control state machine, independent of ROS and HTTP for verification."""

import math
import time
import uuid
from collections import deque
from datetime import datetime
from zoneinfo import ZoneInfo

from .map_view import measured_pose, pose_at

# Map headings of the vehicle front: QR faces +vy; material stations face -vy.
HEADINGS = {2: 'DOWN', 4: 'LEFT', 6: 'RIGHT', 8: 'DOWN'}
BUSY = {'INITIALIZING', 'NAVIGATING', 'WAITING', 'STOPPING'}
LABELS = ['扫码等待', '第一批取料等待', '第一批粗加工等待', '第一批暂存等待',
          '第二批取料等待', '第二批粗加工等待', '第二批码垛等待']


def settings_from(raw):
    if raw.get('blocked_edges'):
        raise ValueError('初版暂未支持封路，请使用全通地图')
    mode = raw.get('mode', 'free')
    if mode not in ('free', 'competition'):
        raise ValueError('未知模式')
    start, goal = raw.get('start'), raw.get('goal')
    if type(start) is not int or type(goal) is not int or start not in range(1, 10) or goal not in range(1, 10):
        raise ValueError('起点与终点必须为1～9')
    heading = raw.get('heading', 'DOWN')
    if mode == 'competition':
        if start not in (1, 3) or goal not in (1, 3):
            raise ValueError('比赛起终点只能为启停区1或2')
        heading = 'DOWN' if start == 1 else 'UP'
    if heading not in ('UP', 'RIGHT', 'DOWN', 'LEFT'):
        raise ValueError('初始朝向不合法')
    align = raw.get('align', False)
    if not isinstance(align, bool) or (align and mode == 'free' and goal not in HEADINGS):
        raise ValueError('只有工作区终点允许作业对齐')
    exit_start = raw.get('exit_start', False)
    if not isinstance(exit_start, bool):
        raise ValueError('起步斜移开关必须为布尔值')
    if mode == 'competition':
        exit_start = True
    elif exit_start and start not in (1, 3):
        raise ValueError('起步斜移仅支持启停区节点1或3')
    return dict(mode=mode, start=start, goal=goal, heading=heading, align=align,
                exit_start=exit_start)


class Session:
    def __init__(self, io, config, lengths, clock=time.monotonic, parking_offset_mm=200):
        self.io, self.cfg, self.lengths, self.clock = io, config, lengths, clock
        self.parking_offset_mm = parking_offset_mm
        self.state, self.reason, self.phase = 'IDLE', '', ''
        self.run_id = self.executor_id = self.owner = ''
        self.settings = None
        self.runtime = self.board = None
        self.runtime_at = self.board_at = self.odom_at = -math.inf
        self.owner_at = self.control_at = -math.inf
        self.started_at = None
        self.finished_at = None
        self.yaw_deg = 0.0
        self.odom = None
        self.binding = self.task_binding = None
        self.anchor = None
        self.segment = None
        self.request_id = ''
        self.accepted = False
        self.routes, self.route_index = [], 0
        self.plan = None
        self.pose = None
        self.trace = deque(maxlen=1200)
        self.events = deque(maxlen=80)
        self.sequence = 0
        self.deadline = self.wait_until = 0.0
        self.pending = {}
        self.epoch = 0
        self.stop_fault = False
        self.last_command = [0.0, 0.0, 0.0]
        self.preflight_reason = '等待规划器、C板反馈'
        self._record('调试工具已启动；请先人工摆车')

    def _record(self, message):
        self.events.append(dict(time=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S'), message=message))

    def _state(self, state, reason=''):
        self.state, self.reason = state, reason
        self._record(reason or state)

    def _zero(self):
        self.last_command = [0.0, 0.0, 0.0]
        self.io.velocity(*self.last_command)

    def heartbeat(self, client):
        if client == self.owner:
            self.owner_at = self.clock()

    def health(self, allow_lock=False):
        now = self.clock()
        if not self.io.exclusive():
            return '存在其他底盘控制发布者或冲突执行端'
        if self.board is None or now - self.board_at > self.cfg['status_timeout_s']:
            return '等待新鲜C板状态'
        for field, text in [('connected', 'C板未连接'), ('status_valid', 'C板状态无效'),
                            ('pc_control', 'C板未切换至PC控制'), ('odometry_valid', 'C板里程无效')]:
            if not self.board.get(field):
                return text
        if not allow_lock and not self.board.get('motion_allowed'):
            return self.board.get('fault_reason') or 'C板运动未获许可'
        return ''

    def command(self, action, raw, client):
        if action == 'stop':
            # Any same-origin observer may stop, but only the owner may start/reset.
            if self.state in BUSY or self.state == 'READY':
                self.stop('用户停止，需重新摆车并复位')
            return
        if action not in ('start', 'reset'):
            raise ValueError('未知操作')
        if self.state in BUSY:
            raise ValueError('当前正在执行操作，请先停止')
        if self.owner and self.owner != client and self.clock() - self.owner_at < self.cfg['browser_timeout_s']:
            raise ValueError('另一页面持有控制权；本页只读，可停止')
        settings = settings_from(raw)
        why = self.health(allow_lock=action == 'reset')
        if why:
            raise ValueError(why)
        if not self.io.services_ready():
            raise ValueError('规划器服务未就绪')
        if self.pending:
            raise ValueError('尚有结果未明确的接口请求，请等待或重启工具后重新摆车')
        if self.phase == 'UNCONFIRMED':
            raise ValueError('停止尚未确认，请检查现场并重新启动工具')
        self.owner, self.owner_at = client, self.clock()
        self.epoch += 1
        self.settings = settings
        self.auto_start = action == 'start'
        self.task_binding = self.binding
        self.anchor = None
        self.request_id, self.accepted, self.plan = '', False, None
        self.trace.clear()
        self.pose = None
        if settings['mode'] == 'competition':
            self.routes = [dict(goal=settings['start'], maneuver='EXIT', wait='')]
            self.routes += [dict(goal=goal, maneuver='', wait=label)
                            for goal, label in zip((2, 4, 6, 8, 4, 6, 8), LABELS)]
            self.routes += [dict(goal=settings['goal'], maneuver='', wait=''),
                            dict(goal=settings['goal'], maneuver='ENTER', wait='')]
        else:
            self.routes = [dict(goal=settings['goal'], maneuver='', wait='')]
            if settings['exit_start']:
                self.routes.insert(0, dict(goal=settings['start'], maneuver='EXIT', wait=''))
        self.route_index = 0
        self.started_at, self.finished_at = None, None
        self.deadline = self.clock() + self.cfg['initialization_timeout_s']
        self._zero()
        self._state('INITIALIZING', '检查执行器并同步人工摆放的位置')
        self.phase = 'GET_RUNTIME'
        self._call('get_runtime', {}, self._begin_reset)

    def _call(self, kind, payload, callback):
        token = uuid.uuid4().hex
        epoch = self.epoch
        self.pending[token] = (self.clock(), kind)

        def done(result, error=None):
            if token not in self.pending:
                return
            self.pending.pop(token)
            if epoch != self.epoch:
                return
            if error:
                self.stop(f'{kind}: {error}，执行结果需查询确认', fault=True)
                return
            callback(result)
        self.io.rpc(kind, payload, done)

    def _begin_reset(self, result):
        rt = result['runtime']
        if rt['source'] != 'REAL':
            self.stop('工具要求真实路径执行器，拒绝模拟来源', fault=True)
            return
        if rt['busy'] or not rt['quiescent'] or not rt['motion_released']:
            self.stop('规划器尚未释放控制，先停止并确认现场', fault=True)
            return
        self.executor_id = rt['executor_session_id']
        self.run_id = uuid.uuid4().hex
        self.phase = 'RESET_CONTEXT'
        self._call('reset_context', dict(expected_run_id=rt['run_id'], run_id=self.run_id,
                   reset_id=uuid.uuid4().hex, home_node=self.settings['start'],
                   initial_heading=self.settings['heading']), self._reset_done)

    def _reset_done(self, result):
        if not result.get('accepted'):
            self.stop('初始化被拒绝：' + result.get('reason', ''), fault=True)
            return
        rt = result['runtime']
        if rt['run_id'] != self.run_id or rt['executor_session_id'] != self.executor_id:
            self.stop('复位返回身份不一致', fault=True)
            return
        self.runtime, self.runtime_at = rt, self.clock()
        self.odom_at = -math.inf
        self.pose = pose_at(self.settings['start'], self.settings['heading'])
        self.pose['valid'] = False
        self.phase = 'WAIT_ODOMETRY'

    def update_runtime(self, runtime):
        self.runtime, self.runtime_at = runtime, self.clock()

    def _binding_changed(self, binding):
        if self.binding is not None and binding != self.binding:
            self.anchor = None
            self.trace.clear()
            if self.pose:
                self.pose['valid'] = False
            if self.state in ('INITIALIZING', 'NAVIGATING', 'WAITING', 'READY'):
                self.stop('坐标/控制许可已变化；重新选择当前位置和朝向后开始', fault=True)
        self.binding = binding

    def update_board(self, board):
        self._binding_changed((board['odom_session_id'], board['motion_epoch']))
        self.board, self.board_at = board, self.clock()

    def update_odom(self, odom):
        binding = (odom['odom_session_id'], odom['motion_epoch'])
        # An old DDS sample must not roll the session back after status changed.
        if self.board and binding != (self.board['odom_session_id'], self.board['motion_epoch']):
            return
        self._binding_changed(binding)
        values = [odom[k] for k in ('x_m', 'y_m', 'yaw_rad', 'yaw_deg',
                                   'vx_mps', 'vy_mps', 'wz_rad_s')]
        if not odom['valid'] or not odom['origin_initialized'] or not all(math.isfinite(v) for v in values):
            self.odom_at = -math.inf
            if self.state in ('NAVIGATING', 'WAITING', 'READY'):
                self.stop('收到无效二维里程', fault=True)
            return
        self.odom, self.yaw_deg, self.odom_at = odom, odom['yaw_deg'], self.clock()
        if self.state == 'INITIALIZING' and self.phase == 'WAIT_ODOMETRY' and self.anchor is None:
            self.anchor = dict(odom)
        if self.anchor is not None and self.settings:
            self.pose = measured_pose(odom, self.anchor, self.settings['start'],
                                      self.settings['heading'], self.lengths,
                                      self.parking_offset_mm if self.settings['exit_start'] else 0)
            point = [self.pose['x'], self.pose['y']]
            if not self.trace or math.dist(point, self.trace[-1]) >= 1.:
                self.trace.append(point)

    def _dispatch(self):
        self.phase = ''
        self.started_at = self.started_at if self.started_at is not None else self.clock()
        self.request_id = uuid.uuid4().hex
        self.accepted, self.sequence, self.plan = False, 0, None
        self.terminal = None
        self.control_at = self.clock()
        route = self.routes[self.route_index]
        goal, maneuver = route['goal'], route['maneuver']
        align = not maneuver and (bool(route['wait']) or
                                  self.settings['mode'] == 'free' and self.settings['align'])
        description = (f'斜移出区 → 节点{goal}角点' if maneuver == 'EXIT' else
                       f'斜移入区 → 节点{goal}启停区' if maneuver == 'ENTER' else f'→ 节点{goal}')
        self.segment = None
        self._state('NAVIGATING', f'导航 {self.route_index + 1}/{len(self.routes)} {description}')
        self._call('plan', dict(run_id=self.run_id, request_id=self.request_id,
                   stage=0, start=0, goal=goal, current_heading='', align_at_goal=align,
                   goal_heading=HEADINGS[goal] if align else '', parking_maneuver=maneuver,
                   parking_offset_mm=self.parking_offset_mm if maneuver else 0), self._planned)

    def _planned(self, result):
        plan = result['plan']
        if plan['request_id'] != self.request_id or plan['run_id'] != self.run_id:
            self.stop('规划响应身份不一致', fault=True)
        elif plan['status'] != 'ok':
            self.stop('规划失败：' + plan['reason'], fault=True)
        else:
            self.plan, self.accepted, self.control_at = plan, True, self.clock()
            route = self.routes[self.route_index]
            if route['maneuver']:
                origin = dict(x_m=0., y_m=0., yaw_rad=0., yaw_deg=0., valid=True)
                offsets = (self.parking_offset_mm, 0) if route['maneuver'] == 'EXIT' else (0, self.parking_offset_mm)
                points = [measured_pose(origin, origin, route['goal'], 'RIGHT', self.lengths, offset)
                          for offset in offsets]
                plan['display_points'] = [[point['x'], point['y']] for point in points]
            if self.terminal:
                self._terminal(*self.terminal)

    def feedback(self, msg, fresh=True):
        if self.state != 'NAVIGATING' or msg['run_id'] != self.run_id or msg['request_id'] != self.request_id:
            return
        if msg['executor_session_id'] != self.executor_id or msg['source'] != 'REAL':
            return
        if not fresh or msg['sequence'] <= self.sequence:
            return
        self.sequence, self.control_at = msg['sequence'], self.clock()
        if msg['state'] in ('COMPLETED', 'ABORTED'):
            self._zero()
            self.terminal = (msg['state'], msg.get('reason', ''))
            if self.accepted:
                self._terminal(*self.terminal)
        elif msg['state'] == 'EXECUTING' and self.accepted and not self.terminal:
            cmd = msg['command']
            values = [cmd['vx_mps'], cmd['vy_mps'], cmd['omega_deg_s']]
            if not all(math.isfinite(v) for v in values):
                self.stop('规划器速度无效', fault=True)
                return
            binding = (cmd.get('odom_session_id', ''), cmd.get('motion_epoch', 0))
            if binding != self.task_binding or binding != self.binding:
                self.stop('旧任务速度绑定已失效', fault=True)
                return
            self.last_command = values
            self.io.velocity(*values, binding=binding)

    def _terminal(self, state, reason):
        if self.phase == 'VERIFY_COMPLETION':
            return
        if state != 'COMPLETED':
            self.stop(reason or '路线中止', fault=True)
            return
        self.phase = 'VERIFY_COMPLETION'
        self._call('get_runtime', {}, self._verify_completion)

    def _verify_completion(self, result):
        rt = result['runtime']
        route = self.routes[self.route_index]
        goal = route['goal']
        expected_heading = self.plan['end_heading']
        if not (rt['run_id'] == self.run_id and rt['executor_session_id'] == self.executor_id
                and rt['last_task_id'] == self.request_id and rt['last_state'] == 'COMPLETED'
                and rt['current_node'] == goal and rt['position_known'] and rt['heading_known']
                and rt['current_heading'] == expected_heading and not rt['busy']
                and rt['quiescent'] and rt['motion_released']):
            self.stop('导航完成核对失败', fault=True)
            return
        self._zero()
        self.phase = ''
        if route['wait']:
            self.wait_until = self.clock() + self.cfg['wait_s']
            self._state('WAITING', route['wait'])
        elif self.route_index + 1 < len(self.routes):
            self.route_index += 1
            self._dispatch()
        else:
            self.finished_at = self.clock()
            self._state('FINISHED', '路线演练完成；底盘输出已归零')

    def progress(self, msg, fresh=True):
        if not self.plan or msg['run_id'] != self.run_id or msg['request_id'] != self.request_id or msg['executor_session_id'] != self.executor_id:
            return
        if self.state not in ('NAVIGATING', 'WAITING', 'FINISHED') or not fresh:
            return
        if self.state in ('WAITING', 'FINISHED') and msg['state'] != 'COMPLETED':
            return
        self.segment = msg

    def stop(self, reason, fault=False):
        if self.state == 'STOPPING':
            self.stop_fault |= fault
            return
        self.epoch += 1
        self.finished_at = self.clock()
        self.accepted, self.stop_fault = False, fault
        self._zero()
        self.phase = 'QUERY_STOP'
        self.deadline = self.clock() + self.cfg['service_timeout_s'] * 2
        if self.pose:
            self.pose['valid'] = False
        self._state('STOPPING', reason)
        self._query_stop()

    def _query_stop(self):
        if self.phase != 'QUERY_STOP' or self.pending:
            return
        self.phase = 'STOP_QUERY_PENDING'
        self._call('get_runtime', {}, self._stop_runtime)

    def _stop_runtime(self, result):
        rt = result['runtime']
        # Use the queried identity: a reset in flight may have just finished.
        if rt['busy']:
            self.phase = 'STOP_RPC'
            self._call('stop', dict(run_id=rt['run_id'], task_id=rt['active_task_id']), self._stopped_rpc)
        elif rt['quiescent'] and rt['motion_released']:
            self.run_id, self.executor_id = rt['run_id'], rt['executor_session_id']
            self.phase = ''
            self._state('FAULT' if self.stop_fault else 'STOPPED', self.reason)
        else:
            self.phase = 'QUERY_STOP'

    def _stopped_rpc(self, result):
        if not result.get('accepted'):
            self.reason = '停止未被确认：' + result.get('reason', '')
        self.phase = 'QUERY_STOP'

    def tick(self):
        now = self.clock()
        if self.run_id:
            self.io.lease(self.run_id)
        self.preflight_reason = self.health()
        if self.state in ('INITIALIZING', 'NAVIGATING', 'WAITING', 'READY'):
            if now - self.owner_at > self.cfg['browser_timeout_s']:
                self.stop('控制页面失联，需复位后重新开始', fault=True)
            elif self.state != 'INITIALIZING' and self.preflight_reason:
                self.stop(self.preflight_reason, fault=True)
            elif self.state in ('NAVIGATING', 'WAITING'):
                if now - self.odom_at > self.io.odom_timeout:
                    self.stop('真实里程反馈超时', fault=True)
                elif now - self.runtime_at > self.cfg['service_timeout_s']:
                    self.stop('执行器运行查询超时', fault=True)
                elif self.runtime and self.runtime['executor_session_id'] != self.executor_id:
                    self.stop('规划器进程重启，禁止继续旧轮次', fault=True)
        if self.state == 'INITIALIZING':
            if now > self.deadline:
                self.stop('初始化超时：' + (self.preflight_reason or self.phase), fault=True)
            elif self.phase == 'WAIT_ODOMETRY' and not self.preflight_reason and self.runtime:
                rt = self.runtime
                if (rt['run_id'] == self.run_id and rt['odometry_ready'] and self.anchor is not None
                        and self.task_binding == self.binding and now-self.odom_at < self.io.odom_timeout):
                    self.phase = 'WAIT_MAP'
            if self.phase == 'WAIT_MAP':
                self.io.map(self.run_id, 1, [])
                rt = self.runtime
                if rt and rt['run_id'] == self.run_id and rt['map_ready'] and rt['map_revision'] == 1 and not rt['blocked_edges'] and rt['config_ready']:
                    self.pose['valid'] = True
                    if self.auto_start:
                        self._dispatch()
                    else:
                        self.phase = ''
                        self._state('READY', '复位完成，可以启动')
        elif self.state == 'NAVIGATING' and self.accepted and self.phase != 'VERIFY_COMPLETION':
            if now - self.control_at > self.cfg['route_control_timeout_s']:
                self.stop('导航速度反馈超时', fault=True)
        elif self.state == 'WAITING' and now >= self.wait_until:
            self.route_index += 1
            self._dispatch()
        elif self.state == 'STOPPING':
            if now > self.deadline:
                self._state('FAULT', '停止状态未确认，请检查执行器并重新启动工具；' + self.reason)
                self.phase = 'UNCONFIRMED'
            else:
                self._query_stop()
        # Pending service calls expire without authorizing any retry of movement.
        for token, (at, kind) in list(self.pending.items()):
            if now - at > self.cfg['service_timeout_s']:
                # Keep unresolved movement/reset calls latched: no false stopped/ready.
                if self.state not in ('STOPPING', 'FAULT'):
                    self.stop(f'{kind}响应超时，结果未知，请检查后重启工具', fault=True)
                break
        if self.state != 'NAVIGATING' or not self.accepted or self.phase == 'VERIFY_COMPLETION':
            self._zero()
        if self.pose and now - self.odom_at > self.io.odom_timeout:
            self.pose['valid'] = False

    def snapshot(self):
        now = self.clock()
        return dict(state=self.state, reason=self.reason, phase=self.phase,
                    owner=self.owner, settings=self.settings, run_id=self.run_id,
                    route_index=self.route_index, route_count=len(self.routes),
                    goal=self.routes[self.route_index]['goal'] if self.routes else None,
                    parking_maneuver=self.routes[self.route_index]['maneuver'] if self.routes else '',
                    parking_offset_mm=self.parking_offset_mm,
                    plan=self.plan, pose=self.pose, trace=list(self.trace),
                    events=list(self.events), board=self.board,
                    board_fresh=now - self.board_at < self.cfg['status_timeout_s'],
                    odometry_fresh=now - self.odom_at < self.io.odom_timeout,
                    odometry=self.odom, yaw_deg=self.yaw_deg, segment=self.segment,
                    odom_session_id=self.binding[0] if self.binding else '',
                    motion_epoch=self.binding[1] if self.binding else 0,
                    command=self.last_command,
                    elapsed_s=max(0, (self.finished_at or now) - self.started_at) if self.started_at is not None else 0,
                    wait_s=max(0, self.wait_until - now) if self.state == 'WAITING' else 0,
                    preflight_reason=self.preflight_reason,
                    unresolved=bool(self.pending) or self.phase == 'UNCONFIRMED')
