"""
淘晶驰串口屏 Python 驱动示例
========================================
适用：TJC4827T143 4.3寸，t0/t1 文本控件
波特率：9600（出厂默认）
依赖：pip install pyserial

接线：
    串口模块 TX -> 屏幕 RX
    串口模块 RX -> 屏幕 TX
    GND        -> GND
    5V         -> 5V
"""

import serial
import time


class TJCScreen:
    """淘晶驰串口屏驱动"""

    def __init__(self, port='COM5', baud=9600):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)  # 等屏幕就绪

    def _send(self, cmd):
        """发送一条指令（自动追加 3 个 0xFF）"""
        if isinstance(cmd, str):
            cmd = cmd.encode()
        self.ser.write(cmd + b'\xff\xff\xff')
        time.sleep(0.1)

    def set_text(self, control, text):
        """给文本控件赋值：set_text('t0', '231+232')"""
        self._send(f'{control}.txt="{text}"')

    def set_number(self, control, value):
        """给数字控件赋值：set_number('n0', 123)"""
        self._send(f'{control}.val={value}')

    def get_text(self, control):
        """读取文本控件内容"""
        self.ser.reset_input_buffer()
        self._send(f'get {control}.txt')
        time.sleep(0.2)
        resp = self.ser.read(self.ser.in_waiting or 100)
        # 返回格式：0x70 + 数据 + FF FF FF
        try:
            return resp[1:-3].decode()
        except Exception:
            return None

    def show_page(self, page_id):
        """切换页面：show_page(0)"""
        self._send(f'page {page_id}')

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    # ===== 使用示例 =====
    screen = TJCScreen(port='COM5', baud=9600)

    # 显示任务码
    screen.set_text('t0', '231+232')
    screen.set_text('t1', '423+312')

    # 读回验证
    print('t0 =', screen.get_text('t0'))
    print('t1 =', screen.get_text('t1'))

    screen.close()
