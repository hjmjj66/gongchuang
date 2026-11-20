"""Closing an active arm controller flushes stop/abort on the actual PTY link."""
import os
import time
import unittest
from agv_cboard.protocol import ARM_CMD, ARM_JOG, FrameParser, encode_arm, encode_chassis, encode_jog
from agv_cboard.transport import SerialTransport


class ArmShutdownTests(unittest.TestCase):
    def test_shutdown_writes_zero_jog_and_abort_before_serial_close(self):
        master,slave=os.openpty()
        os.set_blocking(master,False)
        transport=SerialTransport(os.ttyname(slave),50.,.1,lambda now:None,
            lambda *args:None,lambda *args:None,lambda now:encode_chassis(0.,0.,0.,0),
            lambda *args:None,get_extra=lambda now:[encode_jog(.1,2.,0.,1)])
        parser=FrameParser()
        frames=[]
        def read():
            try:
                frames.extend(parser.feed(os.read(master,4096)))
            except BlockingIOError:
                pass
        transport.start()
        try:
            until=time.monotonic()+2.
            while not any(mid==ARM_JOG for mid,_ in frames) and time.monotonic()<until:
                read()
                time.sleep(.002)
            self.assertTrue(any(mid==ARM_JOG and payload[12]==1 for mid,payload in frames))
            transport.stop((encode_jog(),encode_arm(0)))
            read()
            self.assertEqual(frames[-2][0],ARM_JOG)
            self.assertEqual(frames[-2][1][12],0)
            self.assertEqual(frames[-1],(ARM_CMD,b'\0\0\0\0'))
        finally:
            transport.stop()
            os.close(master)
            os.close(slave)


if __name__=='__main__':
    unittest.main()
