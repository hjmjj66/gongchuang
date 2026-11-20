"""Full-run orchestration; route ownership and release checks stay in route_debug."""

from datetime import datetime
from zoneinfo import ZoneInfo
import uuid

from agv_mission.stages import Stage, WORK, batch_spec, expected_steps
from agv_mission.task_code import parse_task_code
from route_debug.session import Session, BUSY, settings_from

AREAS = {'raw': '原料区', 'coarse': '粗加工区', 'storage': '暂存区'}
WORK_STAGE = {2: Stage.RAW_BATCH_1, 3: Stage.COARSE_BATCH_1, 4: Stage.STORAGE_PLACE_1,
              5: Stage.RAW_BATCH_2, 6: Stage.COARSE_BATCH_2, 7: Stage.STORAGE_STACK_2}
PHASES = ['启停区出区', '前往二维码区', '前往原料区', '前往粗加工区', '前往暂存区',
          '前往原料区', '前往粗加工区', '前往暂存区', '返回最终停车区', '最终停车区入区']


def run_options(raw):
    mode = raw.get('mode', 'web')
    if mode not in ('web', 'screen'):
        raise ValueError('配置模式须为网页配置或显示屏配置')
    switches = raw.get('real_actions', dict.fromkeys(AREAS, True))
    if not isinstance(switches, dict) or set(switches) != set(AREAS) or any(type(v) is not bool for v in switches.values()):
        raise ValueError('三区域真实取放开关须完整且为布尔值')
    return mode, dict(switches)


class CompetitionSession(Session):
    QR_WAIT_REASON = '扫码区已停车，持续等待，首帧有效任务码即采用'

    def __init__(self, *args, **kwargs):
        self.log = []
        self.results = []
        self.locked = False
        self.config_mode = 'web'
        self.real_actions = dict.fromkeys(AREAS, True)
        self.code = None
        self.first_layer = {}
        self.radar = False
        self.radar_map = None
        self.current_action = ''
        super().__init__(*args, **kwargs)
        self.trace = []

    @property
    def batch(self):
        return 1 if 2 <= self.route_index <= 4 else 2 if 5 <= self.route_index <= 7 else 0

    def _record(self, message):
        row = dict(time=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),
                   batch=self.batch, message=message)
        self.log.append(row)
        super()._record(message)
        self.events[-1]['batch'] = self.batch

    def command(self, action, raw, client):
        if action in ('camera_open', 'qr_camera_open', 'camera_close'):
            if self.state != 'IDLE' or self.locked:
                raise ValueError('仅待命时可手动打开或关闭相机预览；比赛中由流程管理相机')
            if action == 'camera_open':
                self.io.open_idle_preview()
            elif action == 'qr_camera_open':
                self.io.open_idle_preview('qr')
            else:
                self.io.release_camera()
            return
        if action == 'stop':
            if self.state == 'IDLE':
                self.io.release_camera()
            return super().command(action, raw, client)
        if action not in ('start', 'reset'):
            raise ValueError('仅支持整场开始、停止和复位')
        if self.state in BUSY:
            raise ValueError('本轮正在执行，请先停止并等待释放确认')
        if self.owner and self.owner != client and self.clock()-self.owner_at < self.cfg['browser_timeout_s']:
            raise ValueError('另一页面持有控制权')
        if self.pending or self.phase == 'UNCONFIRMED':
            raise ValueError('停止或接口结果尚未确认，禁止复位和开始')
        if not self.io.work_released():
            raise ValueError('区域执行端尚未确认释放：'+(self.io.work_health(False) or '等待停止确认或新鲜空闲反馈'))
        if action == 'reset':
            self.io.screen_stop()
            self.io.stop_qr()
            self.io.release_camera()
            self._zero()
            self.locked = False
            self.settings = None
            self.code = self.qr_result = self.work_status = None
            self.first_layer = {}
            self.radar = False
            self.radar_map = None
            self.routes = []
            self.route_index = 0
            self.started_at = self.finished_at = None
            self.screen_routes = None
            self.screen_zones = None
            self.start_not_before = 0.
            self.plan = self.pose = self.anchor = self.segment = None
            self.trace.clear()
            self.current_action = self.phase = self.run_id = ''
            self.screen_needs_reset = False
            self.owner, self.owner_at = client, self.clock()
            self._state('IDLE', '软件复位完成；人工恢复车辆、物料和机构后可重新配置')
            return
        if self.locked:
            raise ValueError('本轮已结束，须人工恢复现场并先复位')
        mode, switches = run_options(raw)
        settings = dict(raw, mode='screen' if mode == 'screen' else 'competition',
                        work_alignment=True, real_qr=mode == 'screen')
        if mode == 'web':
            settings_from(settings)
        self.io.prepare_work()
        self.config_mode, self.real_actions = mode, switches
        self.log.clear()
        self.events.clear()
        self.results.clear()
        self.code = None
        self.first_layer = {}
        self.current_action = ''
        self.radar_map = None
        self.locked = True
        try:
            super().command(action, settings, client)
        except Exception:
            self.locked = False
            raise
        self._record('本轮配置：' + ('网页配置' if mode == 'web' else '显示屏配置') + '；' +
                     '；'.join(f'{AREAS[a]}：' + ('真实取放' if v else '停留1秒替代') for a, v in switches.items()))
        self._record('关键配置及来源：' + str(self.io.config_display))

    def _initialize(self, settings, auto_start):
        settings['real_qr'] = True
        super()._initialize(settings, auto_start)
        self._record(f'本轮起点节点{settings["start"]}，最终停车节点{settings["goal"]}；禁行路径：{settings["blocked_edges"] or "无"}')

    def health(self, allow_lock=False):
        return super().health(allow_lock) or self.io.work_health(self.state == 'WORKING' and not self.radar)

    def _dispatch(self):
        self.radar = False
        self.current_action = PHASES[self.route_index]
        super()._dispatch()

    def _arrived(self, route):
        try:
            self._arrived_full(route)
        except (RuntimeError, ValueError, OSError) as exc:
            self.stop('到站任务启动失败：'+str(exc), fault=True)

    def _arrived_full(self, route):
        if route['maneuver'] == 'EXIT':
            self._record('已斜移离开启停区')
            if self.io.use_lidar:
                self.radar = True
                self.current_action = '开局雷达扫描'
                self._state('WORKING', '等待真实雷达扫描及地图提交')
                self.io.start_lidar(self.run_id)
            else:
                self._record('use_lidar=false：跳过雷达扫描，仅使用本轮人工禁行地图')
                self.route_index += 1
                self._dispatch()
        elif route['wait'] and route['goal'] != 2:
            stage = WORK_STAGE[self.route_index]
            area, operation, batch, colors, positions = batch_spec(stage, self.code, self.first_layer)
            self._record(f'第{["一", "二"][batch-1]}轮：已到达{AREAS[area]}，工作朝向和导航释放已确认')
            self.current_action = operation
            self._state('WORKING', f'{AREAS[area]}整批作业开始')
            self.io.start_batch(area, operation, colors, positions, self.first_layer,
                                'real' if self.real_actions[area] else 'simulated', uuid.uuid4().hex, self.binding)
        else:
            super()._arrived(route)
            if self.state == 'SCANNING':
                self.current_action = '真实扫码，等待首帧有效任务码'
            if self.state == 'FINISHED':
                if self.config_mode == 'screen':
                    completed = [row for row in self.results if row['outcome'] == 'REAL_COMPLETED']
                    self.io.show_counts(
                        sum(row['operation'] == 'PICK' for row in completed),
                        sum(row['operation'] in ('PLACE', 'STACK') for row in completed))
                self.io.release_camera()
                self._record('整场比赛调试完成：最终停车区入区及执行端释放已确认')

    def _work_done(self):
        if self.radar:
            if self.radar_map is None:
                edges = {tuple(sorted(e)) for e in self.work_status['blocked_edges']}
                self.radar_map = sorted(edges | set(self.settings['blocked_edges']))
                self.deadline = self.clock()+self.cfg['initialization_timeout_s']
                self._record(f'雷达扫描完成；合并禁行地图：{self.radar_map}')
            self.io.map(self.run_id, 2, self.radar_map)
            rt = self.runtime
            if (rt['run_id'] == self.run_id and rt['map_ready'] and rt['map_revision'] == 2 and
                    {tuple(sorted((e['a'], e['b']))) for e in rt['blocked_edges']} == set(self.radar_map)):
                self._record('雷达与人工禁行地图已获规划器确认')
                self.route_index += 1
                self._dispatch()
            elif self.clock() > self.deadline:
                raise RuntimeError('雷达地图提交未获规划器确认')
            return
        stage = WORK_STAGE[self.route_index]
        area, operation, _, colors, positions = batch_spec(stage, self.code, self.first_layer)
        expected = [row[1:] for row in expected_steps(operation, colors, positions)]
        if [tuple(row) for row in self.work_status['confirmed']] != expected:
            raise RuntimeError('区域完成记录与本轮真实扫码任务顺序不符')
        if stage == Stage.STORAGE_PLACE_1:
            self.first_layer = {color: position for _, _, color, position in self.work_status['confirmed']}
            self._record('第一层流程位置记录已建立；' + ('真实放置完成' if self.real_actions[area] else
                         '放置为停留替代，第二轮仍须识别真实同色物料，无视觉结果将等待'))
        self._record(f'{AREAS[area]}整批完成；离站恢复、收纳和适用的返回P0已确认')
        for key in ('entry_alignment', 'final_alignment'):
            if self.work_status.get(key) == 'TIMEOUT_SKIPPED':
                self._record('角度纠偏3秒限时，未确认达标，按区域原有流程继续')
        self.route_index += 1
        self._dispatch()

    def _work_updated(self):
        if not self.radar:
            for row in self.work_status.get('events', []):
                self._record(row)
            for row in self.work_status.get('results', []):
                self.results.append(dict(batch=self.batch, **row))
            label = self.work_status['label']
            if label != self.current_action:
                self.current_action = label
                self._record(label)

    def tick(self):
        super().tick()
        if self.qr_result and self.code is None:
            self.code = parse_task_code(self.qr_result)

    def stop(self, reason, fault=False):
        if self.state != 'STOPPING':
            self._record(f'停止上下文：阶段={PHASES[self.route_index]}；动作={self.current_action}；任务码={self.qr_result}；原因={reason}')
            if fault:
                self._record('故障执行上下文：' + str(self.io.fault_context()))
        super().stop(reason, fault)

    def _state(self, state, reason=''):
        super()._state(state, reason)
        if state in ('STOPPED', 'FAULT'):
            self.io.release_camera()
            self._record('停止释放未确认，请检查执行端' if '未确认' in reason else '停止完成，执行端已释放')

    def snapshot(self):
        state = super().snapshot()
        stage = PHASES[self.route_index]
        if self.state == 'SCANNING':
            stage = '二维码区扫码'
        elif self.state == 'WORKING':
            stage = '开局雷达扫描' if self.radar else {
                'RAW_PICK': '原料区取料', 'COARSE_PROCESS': '粗加工区三放三取',
                'STORAGE_PLACE': '暂存区第一层放置', 'STORAGE_STACK': '暂存区第二层码垛'
            }[WORK[WORK_STAGE[self.route_index]][1]]
        state.update(config_mode=self.config_mode, real_actions=self.real_actions, locked=self.locked,
                     batch=self.batch, stage=stage, action=self.current_action,
                     results=self.results.copy(), log=self.log.copy())
        return state
