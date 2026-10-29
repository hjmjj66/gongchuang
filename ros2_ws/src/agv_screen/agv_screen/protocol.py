"""Validated ROUTE/CFG and TJC text protocol, packaged without HMI dependencies."""

import re

END = b'\xff\xff\xff'


class ScreenMessageParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        while True:
            end = self.buffer.find(b'\r\n')
            if end < 0:
                if len(self.buffer) > 1024:
                    self.buffer[:] = self.buffer[-128:]
                return messages
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 2]
            start = max(line.rfind(b'CFG,'), line.rfind(b'ROUTE'))
            if start < 0:
                continue
            line = line[start:]
            match = re.fullmatch(rb'CFG,([12]),([12])', line)
            if match:
                messages.append(('CFG', (int(match[1]), int(match[2]))))
            elif re.fullmatch(rb'ROUTE(?:,(?:[1-9]|1[0-2]))*', line):
                routes = tuple(int(value) for value in line.split(b',')[1:])
                if len(routes) == len(set(routes)):
                    messages.append(('ROUTE', routes))


def task_commands(raw_code):
    if not re.fullmatch(r'[0-9]{3}(\+[0-9]{3}){3}', raw_code):
        raise ValueError('任务码必须为四组三位数字，用英文 + 连接')
    a, b, c, d = raw_code.split('+')
    return (f'show_number.t0.txt="{a}+{b}"', f'show_number.t1.txt="{c}+{d}"')
