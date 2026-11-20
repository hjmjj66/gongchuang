"""Station alignment without pick/place, using the existing coarse controller."""

import math
import time
import uuid

from agv_perception.work_config import read_work_config

from agv_perception.coarse_control import CoarseTask, Pose, validate_exit_motion_config


class WorkAlignmentTask(CoarseTask):
    def __init__(self, config, task_id, now, *, area='coarse'):
        self.CENTER_NEXT_STATE = 'WORK_EXIT' if area == 'coarse' else 'STOW'
        super().__init__(config, task_id, (2,), (2,), now)


class WorkAlignment:
    def __init__(self, node, path, *, vision=None):
        from agv_interfaces.msg import ArmExecution
        from agv_interfaces.srv import ArmCommand

        self.node = node
        self.config = self._read_config(path)
        self.request_type = ArmCommand.Request
        self.client = node.create_client(ArmCommand, '/C/arm_command')
        self.subscription = node.create_subscription(ArmExecution, '/C/arm_execution', self._arm, 1)
        self.arm, self.arm_at = None, -math.inf
        self.future = None
        self.unknown = False
        self.task = None
        self.abort_id = ''
        self.submitted = set()
        self.binding = None
        if vision is None:
            from coarse_debug.vision import RingVision
            vision = RingVision(self.config['coarse']['vision']['ring_config'], 960, node.get_logger())
        self.vision = vision

    @staticmethod
    def _read_config(path):
        from coarse_debug.config import validate_area

        config = read_work_config(path)
        for area in ('coarse', 'storage'):
            validate_area(config, area)
            validate_exit_motion_config(config[area])
        if config['coarse']['vision']['ring_config'] != config['storage']['vision']['ring_config']:
            raise ValueError('两个区域须使用同一圆环识别配置')
        return config

    def _arm(self, msg):
        stamp = msg.header.stamp
        age = (self.node.get_clock().now().nanoseconds-stamp.sec*1_000_000_000-stamp.nanosec)/1e9
        limit = self.config[self.area if self.task else 'coarse']['execution']['feedback_timeout_s']
        if 0 <= age <= limit:
            self.arm, self.arm_at = msg, time.monotonic()-age

    def start(self, area, task_id, binding):
        self.area, self.binding = area, binding
        self.submitted.clear()
        self.abort_id = ''
        self.task = WorkAlignmentTask(self.config[area], task_id, time.monotonic(), area=area)

    def _check_future(self):
        if self.future is None:
            return
        future, at = self.future
        if not future.done():
            if time.monotonic()-at > self.config[self.area]['execution']['arm_request_timeout_s']:
                self.unknown = True
                raise RuntimeError('机构请求响应超时，结果未知，禁止继续路线')
            return
        self.future = None
        try:
            response = future.result()
        except Exception as exc:
            self.unknown = True
            raise RuntimeError('机构请求结果未知：' + str(exc)) from exc
        if not response.accepted:
            raise RuntimeError('机构请求被拒绝：' + response.reason)

    def tick(self, odom, odom_at):
        self._check_future()
        obs = self.vision.latest()
        now = time.monotonic()
        p = self.config[self.area]
        if self.vision.error:
            raise RuntimeError('圆环识别异常：' + self.vision.error)
        if not self.client.service_is_ready():
            raise RuntimeError('机构服务未就绪')
        if self.arm is None or now-self.arm_at > p['execution']['feedback_timeout_s']:
            raise RuntimeError('机构状态反馈过期')
        if self.arm.state in ('FAILED', 'UNKNOWN'):
            raise RuntimeError('机构状态异常：' + self.arm.reason)
        if self.task.vision_kind and (not self.vision.ready or obs is None or
                now-obs.captured_at > p['vision']['max_frame_age_s']):
            raise RuntimeError('主相机识别图像过期/断流')
        pose = Pose(odom['x_m'], odom['y_m'], odom['yaw_rad'], odom_at,
                    odom['vx_mps'], odom['vy_mps'])
        self.task.tick(now, pose, obs, self.arm)
        request = self.task.arm_request
        if request and request['command_id'] not in self.submitted:
            if self.future is None and self.arm.idle and not self.arm.pending:
                self.submitted.add(request['command_id'])
                self.future = (self.client.call_async(self.request_type(**request,
                    odom_session_id=self.binding[0], motion_epoch=self.binding[1])), now)
            elif now-self.task.entered_at > p['execution']['arm_request_timeout_s']:
                raise RuntimeError('等待机构空闲超时')
        return self.task

    def pause(self):
        if self.task is not None and not self.task.finished:
            self.task.velocity, self.task.arm_velocity = (0., 0., 0.), (0., 0.)

    def resume(self, duration):
        if self.task is not None and not self.task.finished:
            from agv_perception.link_pause import resume_task
            resume_task(self.task, duration)

    def cancel(self):
        if self.task is None or self.task.finished or self.abort_id:
            return
        self.task.stop(time.monotonic(), '路线停止', failed=False)
        self.abort_id = uuid.uuid4().hex
        self.abort_future = self.client.call_async(self.request_type(command_id=self.abort_id, cmd=0))
        self.abort_at = time.monotonic()

    def released(self):
        if self.unknown:
            return False
        if self.task is None:
            return True
        try:
            self._check_future()
        except RuntimeError:
            return False
        if self.abort_id:
            if not self.abort_future.done():
                return False
            try:
                if not self.abort_future.result().accepted:
                    return False
            except Exception:
                return False
            return bool(self.future is None and self.arm and self.arm.command_id == self.abort_id and
                        self.arm.idle and not self.arm.pending and self.arm_at > self.abort_at and
                        time.monotonic()-self.arm_at <= self.config[self.area]['execution']['feedback_timeout_s'])
        return self.task.state == 'DONE' and self.future is None

    def close(self):
        self.vision.close()
