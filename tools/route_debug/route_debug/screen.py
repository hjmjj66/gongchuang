"""Serial transport for the browser's gated screen setup workflow."""

from collections import deque
import time

import serial

from agv_screen.protocol import END, ScreenMessageParser, task_commands


class Screen:
    def __init__(self, port, baudrate):
        self.port, self.baudrate = port, baudrate
        self.ser = None
        self.parser = ScreenMessageParser()
        self.outgoing = deque()
        self.next_send = 0.
        self.accepting = False

    def start(self):
        if self.ser is None:
            connection = serial.Serial(port=None, baudrate=self.baudrate, timeout=0,
                write_timeout=.1, exclusive=True, xonxoff=False, rtscts=False, dsrdtr=False)
            connection.dtr = connection.rts = False
            connection.port = self.port
            try:
                connection.open()
            except Exception:
                connection.close()
                raise
            self.ser = connection
        self.ser.reset_input_buffer()
        self.parser = ScreenMessageParser()
        self.outgoing.clear()
        self.outgoing.extend((*task_commands('000+000+000+000'), 'page select_routine'))
        self.accepting = True

    def stop(self):
        self.accepting = False
        self.outgoing.clear()
        self.parser = ScreenMessageParser()

    def tick(self, callback):
        if self.ser is None:
            return
        data = self.ser.read(min(self.ser.in_waiting, 4096))
        if self.accepting:
            for kind, values in self.parser.feed(data):
                if not self.accepting:
                    break
                callback(kind, values)
        now = time.monotonic()
        if self.outgoing and now >= self.next_send:
            payload = self.outgoing.popleft().encode('ascii') + END
            written = self.ser.write(payload)
            if written != len(payload):
                raise OSError(f'屏幕指令未完整写入：{written}/{len(payload)}')
            self.next_send = now + .1

    def close(self):
        self.stop()
        if self.ser is not None:
            self.ser.close()
            self.ser = None
