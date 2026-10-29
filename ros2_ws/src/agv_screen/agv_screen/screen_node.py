"""Serial UI bridge. Mission owns configuration, countdown and real start."""

from collections import deque
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import serial

from agv_interfaces.msg import MissionState, MissionTaskInfo
from agv_interfaces.srv import ConfigureMission
from .protocol import END, ScreenMessageParser, task_commands


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = ScreenNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (serial.SerialException, OSError, ValueError) as exc:
        rclpy.logging.get_logger('screen_node').error(f'屏幕通信停止：{exc}')
        result = 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


class ScreenNode(Node):
    def __init__(self, **kwargs):
        super().__init__('screen_node', **kwargs)
        self.ser = None
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        descriptor = ParameterDescriptor(read_only=True)
        port = self.declare_parameter(
            'serial_port', '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', descriptor).value
        baud = self.declare_parameter('baudrate', 9600, descriptor).value
        if not port or baud <= 0:
            raise ValueError('serial_port 必须非空，baudrate 必须为正整数')
        self.ser = serial.Serial(port=None, baudrate=baud, timeout=0,
                                 write_timeout=0.1, exclusive=True,
                                 xonxoff=False, rtscts=False, dsrdtr=False)
        self.ser.dtr = self.ser.rts = False
        self.ser.port = port
        self.ser.open()
        self.parser = ScreenMessageParser()
        self.outgoing = deque()
        self.next_send = 0.0
        self.state = None
        self.routes = None
        self.submitted = False
        self.configure_future = None
        self.last_task = None
        self.client = self.create_client(ConfigureMission, '/mission/configure_and_start')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_sub = self.create_subscription(MissionState, '/mission/state', self._on_state, durable)
        self.task_sub = self.create_subscription(MissionTaskInfo, '/mission/task_info', self._on_task, durable)
        self.timer = self.create_timer(0.02, self._tick)
        self.get_logger().info(f'屏幕串口已打开：{port}，{baud}、8N1；等待 mission 状态')

    def _on_state(self, message):
        previous = self.state
        self.state = message
        changed = previous is None or (previous.run_id, previous.state) != (message.run_id, message.state)
        if not changed:
            return
        if message.state == 'WAIT_SETUP':
            self.routes = None
            self.submitted = False
            self.configure_future = None
            self.last_task = None
            self.parser = ScreenMessageParser()
            self.ser.reset_input_buffer()
            self.outgoing.clear()
            self.outgoing.extend((*task_commands('000+000+000+000'), 'page select_routine'))
        elif message.state == 'RUNNING':
            self.get_logger().info(f'比赛开始 run_id={message.run_id}')
        elif message.state == 'FAULT':
            self.get_logger().error('比赛未继续：' + message.reason)
        elif message.state == 'FINISHED':
            # TODO(stage4): receive final successful-pick/place counts from mission,
            # then enter result, wait 300 ms, and update t4/t5. No fake zero counts.
            self.get_logger().info('比赛结束；第四阶段成功抓取/放置统计尚待视觉结果接入，保留任务码显示')

    def _on_task(self, message):
        if not message.valid or self.state is None or message.run_id != self.state.run_id:
            return
        identity = (message.run_id, message.raw_code)
        if identity == self.last_task:
            return
        try:
            commands = task_commands(message.raw_code)
        except ValueError as exc:
            self.get_logger().error(f'拒绝显示任务码：{exc}')
            return
        self.outgoing.extend(commands)
        self.last_task = identity
        self.get_logger().info('已排队显示本轮扫码结果：' + message.raw_code)

    def _handle_message(self, kind, values):
        if self.state is None or self.state.state != 'WAIT_SETUP' or self.submitted:
            return
        if kind == 'ROUTE':
            self.routes = values
            self.get_logger().info('人工禁行路径：' + ('、'.join(map(str, values)) if values else '无'))
            self.get_logger().info('请继续在屏幕选择 start/stop，然后按 OK')
        elif self.routes is None:
            self.get_logger().warning('未收到本轮 ROUTE，请先在路径页按 OK')
            self.outgoing.append('page select_routine')
        elif not self.client.service_is_ready():
            self.get_logger().warning('mission 配置服务尚未就绪，请稍后在起终点页重新按 OK')
            self.outgoing.append('page setup')
        else:
            start, stop = values
            request = ConfigureMission.Request(
                expected_run_id=self.state.run_id, start_zone=start, stop_zone=stop,
                forbidden_routes=list(self.routes))
            self.submitted = True
            self.configure_future = self.client.call_async(request)
            self.get_logger().info(
                f'start{start}→节点{1 if start == 1 else 3}；stop{stop}→节点{1 if stop == 1 else 3}；'
                '已提交配置和启动请求，等待 mission 确认')

    def _tick(self):
        data = self.ser.read(min(self.ser.in_waiting, 4096))
        if data:
            self.get_logger().debug('RX ' + data.hex(' '))
            for kind, values in self.parser.feed(data):
                self._handle_message(kind, values)
        if self.configure_future is not None and self.configure_future.done():
            future, self.configure_future = self.configure_future, None
            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().error(f'配置调用结果未知，未自动重发：{exc}')
            else:
                if response.accepted:
                    self.get_logger().info(response.reason)
                else:
                    self.submitted = False
                    self.outgoing.append('page setup')
                    self.get_logger().error('配置未接受：' + response.reason)
        if self.outgoing and time.monotonic() >= self.next_send:
            command = self.outgoing.popleft()
            payload = command.encode('ascii') + END
            written = self.ser.write(payload)
            if written != len(payload):
                raise OSError(f'屏幕指令未完整写入：{written}/{len(payload)}')
            self.next_send = time.monotonic() + 0.1
            self.get_logger().debug('TX ' + command)
            if command == 'page select_routine':
                self.get_logger().info('屏幕路径页已请求打开，请选择本轮禁行路径并按 OK；请核对旧高亮')

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        return super().destroy_node()


if __name__ == '__main__':
    raise SystemExit(main())
