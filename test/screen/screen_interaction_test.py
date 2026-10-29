#!/usr/bin/env python3
"""屏幕四阶段通信测试；只收发屏幕数据，不启动比赛或控制机器人。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import select
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)
ROUTE_EDGES = {
    1: (7, 4), 2: (4, 1), 3: (7, 8), 4: (4, 5),
    5: (1, 2), 6: (8, 5), 7: (5, 2), 8: (8, 9),
    9: (5, 6), 10: (2, 3), 11: (9, 6), 12: (6, 3),
}
END = b'\xff\xff\xff'


def main():
    args = parse_args()
    try:
        import serial
    except ImportError:
        print('缺少 pyserial，请运行：python3 -m pip install pyserial', flush=True)
        return 1

    # 导入原解析器时也不在屏幕源目录生成 __pycache__。
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT / '工创赛屏幕驱动'))
    from screen_protocol import ScreenMessageParser

    records = Path(__file__).resolve().parent / 'records'
    records.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    log_path = records / f'interaction_{stamp}.jsonl'
    print(f'收发记录：{log_path}', flush=True)

    try:
        with log_path.open('x', encoding='utf-8') as record:
            with serial.Serial(port=None, baudrate=args.baud, timeout=0.05,
                               write_timeout=1, exclusive=True,
                               xonxoff=False, rtscts=False, dsrdtr=False) as ser:
                ser.dtr = False
                ser.rts = False
                ser.port = args.port
                ser.open()
                print(f'已打开 {args.port}，{args.baud}、8N1、无流控。', flush=True)
                print('本测试只打印“比赛开始”，不启动比赛或控制小车。Ctrl+C 退出。', flush=True)
                session = ScreenInteraction(ser, record, ScreenMessageParser)
                session.log('connection', port=args.port, baud=args.baud)
                session.begin_round()
                while not session.finished:
                    session.tick()
                    readers = [ser]
                    if session.state in ('ROUTE', 'TASK', 'GRAB', 'PUT', 'RESULT'):
                        readers.append(sys.stdin)
                    ready, _, _ = select.select(readers, [], [], 0.05)
                    if ser in ready:
                        session.receive(ser.read(min(ser.in_waiting or 1, 4096)))
                    if sys.stdin in ready:
                        line = sys.stdin.readline()
                        if not line:
                            break
                        session.enter(line.strip())
    except (serial.SerialException, OSError) as exc:
        print(f'\n通信失败：{exc}', flush=True)
        print('请检查端口、USB 连接及串口占用；不自动更换波特率。', flush=True)
        return 1
    print('\n测试退出，保留当前屏幕显示。', flush=True)
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT, help='屏幕串口，默认使用已确认的稳定设备路径')
    parser.add_argument('--baud', type=int, default=9600, help='协议规定为 9600；仅在电控确认后更改')
    args = parser.parse_args()
    if args.baud <= 0:
        parser.error('波特率必须为正整数')
    return args


class ScreenInteraction:
    def __init__(self, ser, record, parser_factory):
        self.ser = ser
        self.record = record
        self.parser_factory = parser_factory
        self.finished = False

    def log(self, kind, **values):
        row = {'time_utc': datetime.now(timezone.utc).isoformat(),
               'monotonic_s': time.monotonic(), 'kind': kind, **values}
        self.record.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.record.flush()

    def say(self, message):
        print(message, flush=True)
        self.log('message', text=message)

    def prompt(self, message):
        print(message, end='', flush=True)

    def send(self, command):
        payload = command.encode('ascii') + END
        count = self.ser.write(payload)
        if count != len(payload):
            raise OSError(f'串口未完整写入：{count}/{len(payload)} 字节')
        self.ser.flush()
        self.log('tx', command=command, hex=payload.hex(' '))
        time.sleep(0.1)

    def begin_round(self):
        self.state = 'ROUTE'
        self.routes = None
        self.selection = None
        self.task = None
        self.grab = None
        self.parser = self.parser_factory()
        self.ser.reset_input_buffer()
        self.send('show_number.t0.txt="000+000"')
        self.send('show_number.t1.txt="000+000"')
        self.send('page select_routine')
        self.say('\n[阶段1] 请在屏幕选择不能走的路径，再按 OK；不选表示无人工禁行。')
        self.say('请核对屏幕高亮是否保留了上轮选择，必要时手动取消。')

    def receive(self, data):
        self.log('rx', hex=data.hex(' '))
        for kind, values in self.parser.feed(data):
            self.log('event', message=kind, values=values, state=self.state)
            if kind == 'ROUTE' and self.state in ('ROUTE', 'CFG'):
                self.routes = values
                self.state = 'CFG'
                text = '、'.join(map(str, values)) if values else '无'
                self.say(f'人工禁行路径：{text}')
                if values:
                    edges = '、'.join(f'{a}↔{b}' for a, b in (ROUTE_EDGES[n] for n in values))
                    self.say(f'对应双向封锁：{edges}')
                self.say('[阶段2] 请在屏幕选择 start/stop，再按 OK。')
            elif kind == 'CFG' and self.state == 'CFG':
                self.deadline = time.monotonic() + 2.0
                self.selection = values
                self.state = 'COUNTDOWN'
                start, stop = values
                self.say(f'start{start} → 节点{1 if start == 1 else 3}；'
                         f'stop{stop} → 节点{1 if stop == 1 else 3}')
                self.say('收到 OK，约 2 秒后模拟开始……')
            elif kind == 'CFG' and self.state == 'ROUTE':
                self.say('尚未收到本轮 ROUTE，请先在路径页按 OK；输入 r 回车可回到路径页。')
            else:
                self.log('ignored', message=kind, state=self.state)

    def tick(self):
        if self.state == 'COUNTDOWN' and time.monotonic() >= self.deadline:
            self.state = 'TASK'
            self.say('比赛开始')
            self.prompt('[阶段3] 输入四组三位数字（例如 452+321+254+312），然后回车：')

    def enter(self, value):
        if self.state == 'TASK':
            if not re.fullmatch(r'[0-9]{3}(\+[0-9]{3}){3}', value):
                self.say('格式错误：必须是四组三位数字，用英文 + 连接。')
                self.prompt('请重新输入任务码：')
                return
            self.task = value
            a, b, c, d = value.split('+')
            self.send(f'show_number.t0.txt="{a}+{b}"')
            self.send(f'show_number.t1.txt="{c}+{d}"')
            self.say(f'任务码指令已发送：{a}+{b} / {c}+{d}，请观察屏幕。')
            self.state = 'GRAB'
            self.prompt('[阶段4] 成功抓取数（观察完任务码后输入，再按回车）：')
        elif self.state in ('GRAB', 'PUT'):
            if not re.fullmatch(r'[0-9]+', value):
                self.say('格式错误：次数必须是非负整数，允许 0。')
                self.prompt('请重新输入成功抓取数：' if self.state == 'GRAB' else '请重新输入成功放置数：')
                return
            count = value.lstrip('0') or '0'
            if self.state == 'GRAB':
                self.grab = count
                self.state = 'PUT'
                self.prompt('成功放置数（输入后按回车）：')
            else:
                self.send('page result')
                time.sleep(0.3)
                self.send(f't4.txt="{self.grab}"')
                self.send(f't5.txt="{count}"')
                self.say(f'统计指令已发送：成功抓取数={self.grab}，成功放置数={count}。')
                self.say('请观察结果页，任务码应保持不变；发送完成不代表实屏已验收。')
                self.state = 'RESULT'
                self.prompt('输入 r 回车重新开始一轮；输入 q 回车退出：')
        elif self.state in ('ROUTE', 'RESULT'):
            if value.lower() == 'r':
                self.begin_round()
            elif value.lower() == 'q':
                self.finished = True
            else:
                self.prompt('请输入 r 或 q，然后回车：')


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\n测试已中断，保留当前屏幕显示。', flush=True)
