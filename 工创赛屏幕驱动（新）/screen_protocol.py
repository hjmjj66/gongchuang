"""屏幕串口通信示例；python screen_protocol.py --port COM5 --task 452+321+254+312"""

import argparse
import re
import time


class ScreenMessageParser:
    """解析 ROUTE/CFG/START；返回 (消息类型, 数字元组)，支持拆包与粘包。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        results = []
        while True:
            end = self.buffer.find(b'\r\n')
            if end < 0:
                if len(self.buffer) > 1024:
                    self.buffer[:] = self.buffer[-128:]
                break
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 2]
            # 跳过启动包或残缺报文，从最后一个合法消息前缀重新同步。
            start = max(
                line.rfind(b'START'),
                line.rfind(b'CFG,'),
                line.rfind(b'ROUTE'),
            )
            if start < 0:
                continue
            line = line[start:]
            if line == b'START':
                results.append(('START', ()))
            elif re.fullmatch(rb'CFG,([13]),([13])', line):
                match = re.fullmatch(rb'CFG,([13]),([13])', line)
                results.append(('CFG', (int(match[1]), int(match[2]))))
            elif re.fullmatch(rb'ROUTE(?:,(?:[1-9]|1[0-2]))*', line):
                routes = tuple(int(x) for x in line.split(b',')[1:])
                if len(routes) == len(set(routes)):
                    results.append(('ROUTE', routes))
        return results


class ConfigParser:
    """旧版解析器，仅识别 CFG；保留兼容。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        results = []
        while True:
            start = self.buffer.find(b"CFG,")
            if start < 0:
                self.buffer[:] = self.buffer[-3:]
                break
            del self.buffer[:start]
            end = self.buffer.find(b"\r\n")
            if end < 0:
                if len(self.buffer) > 128:
                    del self.buffer[0]
                    continue
                break
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 2]
            line = line[line.rfind(b"CFG,"):]
            match = re.fullmatch(rb"CFG,([13]),([13])", line)
            if match:
                results.append((int(match[1]), int(match[2])))
        return results


def send_command(ser, command):
    ser.write(command.encode("ascii") + b"\xff\xff\xff")
    ser.flush()


def show_task_code(ser, task_code):
    """输入完整四组三位任务码；保留字符串形式，避免丢失前导零。"""
    if not re.fullmatch(r"[0-9]{3}(\+[0-9]{3}){3}", task_code):
        raise ValueError("任务码格式必须是 452+321+254+312")
    a, b, c, d = task_code.split("+")
    send_command(ser, f'show_number.t0.txt="{a}+{b}"')
    time.sleep(0.1)
    send_command(ser, f'show_number.t1.txt="{c}+{d}"')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--task", help="联调用的任务码；不传则只接收设置")
    args = parser.parse_args()
    import serial

    receiver = ScreenMessageParser()
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        print("等待屏幕消息；本示例只打印设置，不控制机器人运动。")
        print("流程：选路径 → 选启动/停止区 → 按 START → 发任务码")
        while True:
            data = ser.read(min(ser.in_waiting or 1, 4096))
            for kind, values in receiver.feed(data):
                if kind == 'ROUTE':
                    print(f"收到禁行路径：{values if values else '无'}")
                elif kind == 'CFG':
                    print(f"收到设置：启动区={values[0]}，停止区={values[1]}")
                elif kind == 'START':
                    print("收到 START 信号")
                    if args.task:
                        time.sleep(0.3)
                        show_task_code(ser, args.task)
                        print(f"已发送显示指令：{args.task}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
