"""使用真实子进程、终端 PTY 和串口 PTY 验证四阶段流程，无实物控制。"""

import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import time
import unittest


SCRIPT = Path(__file__).with_name('screen_interaction_test.py')
END = b'\xff\xff\xff'


class ScreenInteractionPTYTest(unittest.TestCase):
    def setUp(self):
        self.screen_fd, self.screen_slave = pty.openpty()
        self.terminal_fd, terminal_slave = pty.openpty()
        self.process = subprocess.Popen(
            [sys.executable, '-B', str(SCRIPT), '--port', os.ttyname(self.screen_slave)],
            stdin=terminal_slave, stdout=terminal_slave, stderr=terminal_slave,
        )
        os.close(terminal_slave)
        self.output = b''
        self.commands = b''
        self.expect_output('收发记录：')
        self.log_path = Path(self.expect_output('\n').strip())

    def tearDown(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=3)
        for fd in (self.screen_fd, self.screen_slave, self.terminal_fd):
            os.close(fd)

    def expect_output(self, text, timeout=4):
        needle = text.encode('utf-8')
        deadline = time.monotonic() + timeout
        while needle not in self.output:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.fail(f'未收到终端输出 {text!r}，实际 {self.output.decode("utf-8", errors="replace")!r}')
            ready, _, _ = select.select([self.terminal_fd], [], [], remaining)
            if ready:
                self.output += os.read(self.terminal_fd, 8192)
        end = self.output.index(needle) + len(needle)
        received, self.output = self.output[:end], self.output[end:]
        return received.decode('utf-8', errors='replace')

    def expect_command(self, command, timeout=3):
        needle = command.encode('ascii') + END
        deadline = time.monotonic() + timeout
        while needle not in self.commands:
            remaining = deadline - time.monotonic()
            self.assertGreater(remaining, 0, f'未收到屏幕指令 {command!r}，实际 {self.commands!r}')
            ready, _, _ = select.select([self.screen_fd], [], [], remaining)
            if ready:
                self.commands += os.read(self.screen_fd, 8192)
        end = self.commands.index(needle) + len(needle)
        self.commands = self.commands[end:]

    def enter(self, value):
        os.write(self.terminal_fd, value.encode('utf-8') + b'\n')

    def test_full_workflow_and_all_start_stop_combinations(self):
        for round_number, (start, stop) in enumerate(((2, 1), (1, 1), (1, 2), (2, 2))):
            with self.subTest(start=start, stop=stop):
                self.expect_command('show_number.t0.txt="000+000"')
                self.expect_command('show_number.t1.txt="000+000"')
                self.expect_command('page select_routine')
                self.expect_output('[阶段1]')
                if round_number == 0:
                    os.write(self.screen_fd, b'CFG,1,1\r\n')
                    self.expect_output('尚未收到本轮 ROUTE')
                    # 含启动包、非法路径和非法 CFG；合法 ROUTE 分两次到达。
                    os.write(self.screen_fd, b'\x88\xff\xff\xffROUTE,13\r\nCFG,3,1\r\nROU')
                    os.write(self.screen_fd, b'TE,1,5,12\r\n')
                    self.expect_output('人工禁行路径：1、5、12')
                    self.expect_output('对应双向封锁：7↔4、1↔2、6↔3')
                else:
                    os.write(self.screen_fd, b'ROUTE\r\n')
                    self.expect_output('人工禁行路径：无')
                self.expect_output('[阶段2]')
                started_at = time.monotonic()
                # 同一次串口写入含两条 CFG，后者不得重复启动或覆盖选择。
                os.write(self.screen_fd, f'CFG,{start},{stop}\r\nCFG,1,1\r\n'.encode('ascii'))
                self.expect_output(f'start{start} → 节点{1 if start == 1 else 3}；'
                                   f'stop{stop} → 节点{1 if stop == 1 else 3}')
                self.expect_output('比赛开始')
                self.assertGreaterEqual(time.monotonic() - started_at, 1.95)
                self.expect_output('[阶段3]')
                if round_number == 0:
                    self.enter('12+345+678+901')
                    self.expect_output('格式错误：必须是四组三位数字')
                self.enter('012+345+678+901')
                self.expect_command('show_number.t0.txt="012+345"')
                self.expect_command('show_number.t1.txt="678+901"')
                self.expect_output('[阶段4]')
                if round_number == 0:
                    self.enter('-1')
                    self.expect_output('次数必须是非负整数')
                self.enter('0')
                self.expect_output('成功放置数（输入后按回车）')
                if round_number == 0:
                    self.enter('1.5')
                    self.expect_output('次数必须是非负整数')
                self.enter('3')
                self.expect_command('page result')
                self.expect_command('t4.txt="0"')
                self.expect_command('t5.txt="3"')
                self.expect_output('统计指令已发送：成功抓取数=0，成功放置数=3')
                self.expect_output('输入 r 回车重新开始一轮')
                self.enter('r' if round_number < 3 else 'q')

        self.expect_output('测试退出')
        self.assertEqual(self.process.wait(timeout=3), 0)
        rows = [json.loads(line) for line in self.log_path.read_text(encoding='utf-8').splitlines()]
        self.assertEqual(sum(row.get('text') == '比赛开始' for row in rows), 4)


if __name__ == '__main__':
    unittest.main()
