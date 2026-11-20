"""Host the existing regional debug tasks with real feedback and one shared camera."""

import math
import time
import uuid
from types import SimpleNamespace

from agv_perception.work_config import read_work_config

from agv_perception.arm_servo import ArmServoPublisher
from agv_perception.vision_gate import VisionGate
from agv_perception.work_control import Pose, GantryPose
from agv_perception.raw_control import validate_raw_config
from agv_perception.coarse_control import validate_coarse_config
from agv_perception.storage_control import validate_storage_config
from coarse_debug.task import DebugTask, RawDebugTask, StorageDebugTask
from route_debug.work_alignment import WorkAlignment


LABELS = dict(CAPTURE_REFERENCE='停稳记录本区域P0', ENTER_WORK_POSE='切换原料工作位',
              INITIAL_FOLLOW='底盘固定，机构初始跟随（最多1秒）',
              WAIT_INITIAL='底盘固定，等待真实物料', WAIT_REQUIRED='等待当前任务颜色',
              TRACK_REQUIRED='跟随并对准当前颜色', RETURN_M0='机构返回本批m0，所需颜色出现立即追踪',
              STOP_M0='机构返回m0后停稳确认',
              RESTORE_WORK='恢复原料工作位', STOW_ARM='执行原料收纳', RETURN_REFERENCE='收纳后底盘返回本区域P0并确认停车',
              WORK_ENTRY='切换机构工作位', ALIGN_ENTRY='入站圆环水平纠偏', CENTER_RING2='底盘对准2号圆环',
              PREPOSITION='机构按标定初值预定位', STOP_PREPOSITION='预定位停车确认',
              ALIGN_RING='机构视觉对准当前圆环', STOP_PLACE='停轴并记录实际机构位置',
              DEBUG_RESTORE_WORK='停留替代后真实恢复工作位', RETURN_ITEM='返回已记录的实际取回位置',
              STOP_PICK='实际取回位置停车复核', WORK_EXIT='粗加工结束恢复工作位', ALIGN_EXIT='粗加工结束水平纠偏',
              STOW='执行机构收纳',
              SCAN_FIRST_LAYER='扫描并等待第一层真实同色物料', SCAN_RESTORE_WORK='扫描完成，恢复工作位',
              ALIGN_MATERIAL='对准第一层真实同色物料', DONE='整批动作和离站流程完成')
OPS = {'PICK': '取料', 'PLACE': '放置', 'STACK': '码垛'}


def read_configs(paths):
    config = {area: read_work_config(path)[area] for area, path in paths.items()}
    for area, validate in (('raw', validate_raw_config), ('coarse', validate_coarse_config), ('storage', validate_storage_config)):
        validate(config[area])
    for area in ('coarse', 'storage'):
        if config[area]['vision']['ring_config'] != config['coarse']['vision']['ring_config']:
            raise ValueError('区域圆环配置必须共用同一主相机')
    if config['raw']['vision']['material_config'] != config['storage']['vision']['material_config']:
        raise ValueError('原料与暂存物料配置必须共用同一主相机')
    return config


class BatchWork(WorkAlignment):
    def __init__(self, node, paths, *, vision=None):
        # WorkAlignment owns the fixed-action acknowledgement and abort protocol.
        self._configs = read_configs(paths)
        super().__init__(node, paths, vision=vision or LazyVision(node, self._configs))
        self.servo = ArmServoPublisher(node)
        self.area = 'coarse'
        self.seen = 0
        self.last_reason = ''
        self.last_state = None
        self.motion_at = 0.
        self._vision_gate = VisionGate()

    def _read_config(self, _):
        return self._configs

    def start_batch(self, area, operation, colors, positions, first_layer, execution, task_id, binding):
        self.area, self.binding = area, binding
        self.submitted.clear()
        self.abort_id = ''
        self.seen = 0
        self.last_reason = ''
        self.last_state = None
        self._vision_gate = VisionGate()
        self.operation = operation
        self.task = make_task(self.config[area], area, operation, colors, positions, first_layer,
                              execution, task_id, time.monotonic())
        self.vision.open()
        self.vision.request(self.task.vision_kind or ('material' if area == 'raw' else 'ring'),
                            geometry_target=self.task.vision_target, largest_per_color=area == 'raw')

    def pause(self):
        super().pause()
        if self.binding:
            self.servo.publish(None, self.binding)

    def health(self, active):
        if not self.client.service_is_ready():
            return '机构服务未就绪'
        if self.arm is None or time.monotonic()-self.arm_at > self.config[self.area]['execution']['feedback_timeout_s']:
            return '等待新鲜机构反馈'
        if self.arm.state in ('FAILED', 'UNKNOWN'):
            return '机构异常：' + self.arm.reason
        if not active and (not self.arm.idle or self.arm.pending):
            return '机构尚未空闲或释放'
        return ''

    def tick(self, odom, odom_at):
        self._check_future()
        why = self.health(True)
        if why:
            raise RuntimeError(why)
        if self.vision.error:
            raise RuntimeError('主相机/识别失败：' + self.vision.error)
        p = self.config[self.area]
        task = self.task
        if task.vision_kind:
            self.vision.request(task.vision_kind, geometry_target=task.vision_target,
                                largest_per_color=self.area == 'raw')
        obs = self.vision.latest()
        now = time.monotonic()
        obs = self._vision_gate.filter(task, obs, now, self.area)
        pose = Pose(odom['x_m'], odom['y_m'], odom['yaw_rad'], odom_at, odom['vx_mps'], odom['vy_mps'])
        board = SimpleNamespace(**self.node.session.board)
        if not task.vision_kind or obs is not None:
            task.tick(now, pose, obs, self.arm, GantryPose.from_status(board, self.node.session.board_at))
        self.servo.publish(task, self.binding, hold_zero=self.area == 'storage' and
                           task.state in ('SCAN_FIRST_LAYER', 'ALIGN_RING', 'ALIGN_MATERIAL'))
        self.motion_at = now
        request = task.arm_request
        if request and request['command_id'] not in self.submitted:
            if self.future is None and self.arm.idle and not self.arm.pending:
                self.submitted.add(request['command_id'])
                self.future = (self.client.call_async(self.request_type(**request,
                    odom_session_id=self.binding[0], motion_epoch=self.binding[1])), now)
            elif now-task.entered_at > p['execution']['arm_request_timeout_s']:
                raise RuntimeError('等待机构空闲/停轴释放超时')
        if task.state in ('ERROR', 'STOPPED'):
            raise RuntimeError(task.reason or '区域任务中止')
        return task

    def status(self, task):
        events, results = [], []
        real = task.execution == 'real'
        for operation, index, color, position in task.confirmed[self.seen:]:
            message = (f'已完成第{index}件（颜色{color}，位置{position}）{OPS[operation]}：真实动作完成' if real else
                       f'第{index}件（颜色{color}，位置{position}）{OPS[operation]}已用停留1秒代替')
            events.append(message)
            results.append(dict(operation=operation, index=index, color=color, position=position,
                                execution=task.execution, outcome='REAL_COMPLETED' if real else 'DWELL_COMPLETED'))
        self.seen = len(task.confirmed)
        if task.state in OPS:
            label = f'第{task.item+1}件（颜色{task.colors[task.item]}，位置{task.positions[task.item]}）{OPS[task.state]}：' + ('真实动作执行中' if real else '全轴停止，停留1秒替代中')
        else:
            label = LABELS.get(task.state, task.state)
        if task.state != self.last_state:
            if self.last_state and self.last_state not in OPS:
                events.append(LABELS.get(self.last_state, self.last_state) + '结束')
            self.last_state = task.state
        wait_reason = task.diagnostic.get('wait_reason') or task.reason
        if wait_reason and wait_reason != self.last_reason:
            events.append('等待/执行说明：' + wait_reason)
        self.last_reason = wait_reason
        if wait_reason:
            label += '；' + wait_reason
        return dict(done=task.state == 'DONE', state=task.state, label=label,
                    velocity=[*task.velocity[:2], math.degrees(task.velocity[2])], events=events, results=results,
                    confirmed=task.confirmed.copy(), entry_alignment=task.entry_alignment,
                    final_alignment=getattr(task, 'final_alignment', 'NOT_RUN'), reason=task.reason)

    def cancel(self):
        self.servo.publish(None, self.binding or ('', 0))
        if self.task is None or self.abort_id:
            return
        self.task.stop(time.monotonic(), '整场停止', failed=False)
        self.abort_id = uuid.uuid4().hex
        self.abort_future = self.client.call_async(self.request_type(command_id=self.abort_id, cmd=0))
        self.abort_at = time.monotonic()

    def released(self):
        return bool(super().released() and (self.task is None or self.arm and self.arm.idle and not self.arm.pending and
                    time.monotonic()-self.arm_at <= self.config[self.area]['execution']['feedback_timeout_s']))


def make_task(config, area, operation, colors, positions, first_layer, execution, task_id, now):
    if area == 'raw':
        return RawDebugTask(config, task_id, colors, now, execution=execution)
    if area == 'coarse':
        return DebugTask(config, task_id, positions, now, colors=colors, execution=execution)
    visit = 1 if operation == 'STORAGE_PLACE' else 2
    return StorageDebugTask(config, task_id, positions if visit == 1 else colors, now, visit, first_layer,
                            colors=colors, execution=execution)


class LazyVision:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.camera = None

    def open(self):
        if self.camera is None:
            from agv_perception.work_vision import RingVision
            self.camera = RingVision(self.config['coarse']['vision']['ring_config'], 960, self.node.get_logger(),
                                     self.config['raw']['vision']['material_config'])

    @property
    def error(self):
        return self.camera.error if self.camera else ''

    def latest(self):
        return self.camera.latest() if self.camera else None

    def preview(self):
        return self.camera.preview() if self.camera else None

    def diagnostics(self):
        if self.camera is None:
            return dict(open=False)
        obs = self.camera.latest()
        return dict(open=True, ready=self.camera.ready, error=self.camera.error,
                    kind=obs.kind if obs else '', sequence=obs.sequence if obs else None,
                    frame_age_s=time.monotonic()-obs.captured_at if obs else None,
                    geometry_target=obs.geometry_target if obs else None,
                    **self.camera.metrics())

    def request(self, *args, **kwargs):
        if self.camera:
            self.camera.request(*args, **kwargs)

    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
