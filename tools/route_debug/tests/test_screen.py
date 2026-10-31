"""Real serial framing and browser gating through a newly created PTY."""

import os
import time
import unittest

import serial
from route_debug.screen import Screen


class ScreenTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        self.screen = Screen(os.ttyname(self.slave), 9600)
        self.addCleanup(os.close, self.slave)
        self.addCleanup(os.close, self.master)
        self.addCleanup(self.screen.close)

    def test_stale_bytes_are_dropped_and_each_new_round_opens_path_page(self):
        messages = []
        self.screen.start()
        os.write(self.master, b'ROUTE,1\r\nCFG,2,1\r\n')
        time.sleep(.01)
        self.screen.start()
        self.screen.tick(lambda *message: messages.append(message))
        self.assertEqual(messages, [])
        outgoing = bytearray()
        for _ in range(3):
            self.screen.next_send = 0.
            self.screen.tick(lambda *message: messages.append(message))
            time.sleep(.01)
            try:
                outgoing.extend(os.read(self.master, 4096))
            except BlockingIOError:
                pass
        self.assertEqual(bytes(outgoing),
            b'show_number.t0.txt="000+000"\xff\xff\xff'
            b'show_number.t1.txt="000+000"\xff\xff\xff'
            b'page select_routine\xff\xff\xff')

    def test_split_frames_and_stop_drop_pending_screen_commands(self):
        self.screen.start()
        messages = []
        callback = lambda *message: messages.append(message)
        for data in (b'noiseROU', b'TE,1,5,12\r', b'\nCFG,2,1\r\nCFG,3,1\r\n'):
            os.write(self.master, data)
            time.sleep(.01)
            self.screen.tick(callback)
        self.assertEqual(messages, [('ROUTE', (1, 5, 12)), ('CFG', (2, 1))])
        self.screen.stop()
        os.write(self.master, b'ROUTE,2\r\nCFG,1,1\r\n')
        time.sleep(.01)
        self.screen.tick(callback)
        self.assertEqual(len(messages), 2)
        self.assertFalse(self.screen.outgoing)

    def test_serial_is_exclusive(self):
        self.screen.start()
        other = Screen(os.ttyname(self.slave), 9600)
        self.addCleanup(other.close)
        with self.assertRaises(serial.SerialException):
            other.start()


if __name__ == '__main__':
    unittest.main()
