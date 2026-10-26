"""One owner for the serial port, with bounded I/O and no command backlog."""

import threading
import time

import serial

from .protocol import FrameParser, encode_chassis


class SerialTransport:
    def __init__(self, port, send_rate, reconnect_interval, on_connected,
                 on_disconnected, on_frame, get_command, log, on_sent=None):
        self.port = port
        self.period = 1.0 / send_rate
        self.reconnect_interval = reconnect_interval
        self.on_connected = on_connected
        self.on_disconnected = on_disconnected
        self.on_frame = on_frame
        self.get_command = get_command
        self.log = log
        self.on_sent = on_sent
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='cboard_serial', daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError('C-board serial worker failed to stop')

    @staticmethod
    def _write(port, frame):
        # Treat a short write as a link failure, never append a new frame to it.
        if port.write(frame) != len(frame):
            raise serial.SerialException('short serial write')

    def _run(self):
        last_log = float('-inf')
        connected_once = False
        while not self._stop.is_set():
            port = None
            opened = False
            try:
                port = serial.Serial(
                    self.port, baudrate=115200, timeout=0, write_timeout=0.02,
                    exclusive=True,
                )
                port.reset_input_buffer()
                port.reset_output_buffer()
                opened = True
                connected_once = True
                self.on_connected(time.monotonic())
                self.log('info', f'Connected to C board: {self.port}')
                parser = FrameParser()
                next_send = time.monotonic()
                while not self._stop.is_set():
                    now = time.monotonic()
                    if now >= next_send:
                        frame = self.get_command(now)
                        self._write(port, frame)
                        if self.on_sent is not None:
                            self.on_sent(frame, time.monotonic())
                        next_send += self.period
                        # Keep cadence without a catch-up burst after an overrun.
                        completed = time.monotonic()
                        if next_send <= completed:
                            next_send = completed + self.period
                    # Bound each RX batch so TX deadlines are checked often.
                    data = port.read(min(max(port.in_waiting, 1), 1024))
                    for msg_id, payload in parser.feed(data, time.monotonic()):
                        self.on_frame(msg_id, payload, time.monotonic())
                    self._stop.wait(min(0.002, max(0.0, next_send - time.monotonic())))
            except (serial.SerialException, OSError) as exc:
                now = time.monotonic()
                # Missing hardware on first start is a wait, not a reset fault.
                if opened or connected_once:
                    self.on_disconnected(now, f'serial failure: {exc}')
                if now - last_log >= 5.0:
                    self.log('warning', f'C board unavailable ({self.port}): {exc}')
                    last_log = now
            except Exception as exc:
                # An unexpected worker failure must never leave cached motion live.
                self.on_disconnected(time.monotonic(), f'serial worker error: {exc}')
                self.log('error', f'C-board worker error: {exc}')
            finally:
                if port is not None:
                    if opened:
                        try:
                            self._write(port, encode_chassis(0.0, 0.0, 0.0, 0))
                        except (serial.SerialException, OSError):
                            pass
                    try:
                        port.close()
                    except OSError:
                        pass
            self._stop.wait(self.reconnect_interval)
