"""交互式屏幕联调：选区 → 任务码 → 正确次数。"""

import argparse
import re
import time

from screen_protocol import ScreenMessageParser


def send(ser, command):
    ser.write(command.encode('ascii') + b'\xff\xff\xff')
    ser.flush()
    print('  发送:', command)
    time.sleep(0.1)


def send_task(ser, task):
    if not re.fullmatch(r'[0-9]{3}(\+[0-9]{3}){3}', task):
        raise ValueError('请输入四组三位数字，例如 452+321+254+312')
    a, b, c, d = task.split('+')
    send(ser, f'show_number.t0.txt="{a}+{b}"')
    send(ser, f'show_number.t1.txt="{c}+{d}"')


def send_counts(ser, grab, put, enter=False):
    if type(grab) is not int or type(put) is not int or grab < 0 or put < 0:
        raise ValueError('次数必须是非负整数')
    if enter:
        send(ser, 'page result')
        time.sleep(0.3)
    send(ser, f't4.txt="{grab}"')
    send(ser, f't5.txt="{put}"')


def ask_task():
    while True:
        value = input('\n请输入任务码（四组三位数，用加号连接）：').strip()
        if re.fullmatch(r'[0-9]{3}(\+[0-9]{3}){3}', value):
            return value
        print('格式错误，需要四组三位数字，用英文加号连接。')


def ask_counts():
    while True:
        value = input('\n请输入抓取次数和放置次数（两个数字，用空格隔开）：').strip()
        if re.fullmatch(r'[0-9]+\s+[0-9]+', value):
            return tuple(map(int, value.split()))
        print('例如输入：3 2，两项之间用空格隔开。')


def choose_port(serial):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    print('\n可用串口：')
    for i, port in enumerate(ports, 1):
        print(f'  {i}. {port.device}  {port.description}')
    if not ports:
        print('  未发现串口。请接好屏幕 USB，检查驱动及供电。')
    while True:
        value = input('输入串口序号或端口名称（如 COM5）：').strip()
        if value.isdigit() and 1 <= int(value) <= len(ports):
            return ports[int(value) - 1].device
        if value:
            return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='不填则交互选择串口')
    parser.add_argument('--baud', type=int, default=9600)
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print('缺少依赖，请先运行：python -m pip install pyserial')
        return
    port = args.port or choose_port(serial)
    try:
        with serial.Serial(port, args.baud, timeout=0.1) as ser:
            print(f'\n已打开 {port}，波特率 {args.baud}。Ctrl+C 退出。')
            print('先关闭串口助手及其他占用此串口的程序。')
            while True:
                input('\n按回车开始一轮测试，程序将屏幕切回 select_routine 页。')
                ser.reset_input_buffer()
                receiver = ScreenMessageParser()
                send(ser, 'show_number.t0.txt="000+000"')
                send(ser, 'show_number.t1.txt="000+000"')
                send(ser, 'page select_routine')
                print('\n请选择不能走的路径，再按 OK；不选表示没有禁行路径。')
                print('随后在 setup 页选择启动区和停止区，再按 OK。')
                selected = None
                routes = None
                while selected is None:
                    data = ser.read(min(ser.in_waiting or 1, 4096))
                    for kind, values in receiver.feed(data):
                        if kind == 'ROUTE':
                            routes = values
                            print('\n收到禁行路径：' + ('、'.join(map(str, routes)) if routes else '无'))
                            print('请继续选择启动区和停止区，按 OK。')
                        elif kind == 'CFG':
                            if routes is None:
                                print('尚未收到 ROUTE，请先在禁行路径页按 OK。')
                            else:
                                selected = values
                print(f'\n收到你的选择：启动区 {selected[0]}，停止区 {selected[1]}。')
                time.sleep(0.3)
                task = ask_task()
                send_task(ser, task)
                print('请观察第二页的任务码。')
                input('确认第二页后按回车，进入次数发送步骤。')
                grab, put = ask_counts()
                send_counts(ser, grab, put, enter=True)
                print('请观察第三页：任务码应保持相同，次数应更新。')
                while True:
                    action = input('\n回车继续更新次数；输入 r 重新选区；输入 q 退出：').strip().lower()
                    if action == 'q':
                        return
                    if action == 'r':
                        break
                    if action:
                        print('请输入 r、q 或直接回车。')
                        continue
                    grab, put = ask_counts()
                    send_counts(ser, grab, put)
    except serial.SerialException as exc:
        print(f'\n串口错误：{exc}\n检查端口名称、连接，以及串口是否被其他程序占用。')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n测试结束。')
