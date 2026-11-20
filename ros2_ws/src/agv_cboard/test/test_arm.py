"""Exercise real encoders and arm operation tracking without hardware."""
import struct
import unittest

from agv_cboard.arm_controller import ArmController
from agv_cboard.protocol import ACK, ARM_CMD, ARM_JOG, FrameParser, encode_arm, encode_jog


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmController(ack_timeout=.5, action_timeout=10.)
        self.binding = ('test-session', 7)

    def submit(self, **operation):
        accepted, reason = self.arm.submit('test-command', self.binding, **operation)
        self.assertTrue(accepted, reason)
        frame, = self.arm.frames(1., True, self.binding)
        self.arm.sent(frame, 1.)
        return frame

    def ack(self, result=0):
        self.arm.ack(ACK, struct.pack('<BBH', ARM_CMD, result, 0))

    def test_reconnect_returns_to_high_work_pose_then_replays_original_action(self):
        self.submit(cmd=5)
        self.arm.pause_link(1.1)
        self.assertEqual(self.arm.frames(20., True, self.binding), [])
        self.arm.status(1, 20.)
        self.arm.resume_link(20.)
        self.assertEqual(self.arm.frames(20., True, self.binding), [])
        self.arm.status(0, 20.1)
        self.assertEqual(self.arm.frames(20.1, True, self.binding), [encode_arm(3)])
        self.arm.sent(encode_arm(3), 20.1)
        self.ack()
        self.arm.status(1, 20.2)
        self.arm.status(0, 20.3)
        self.assertTrue(self.arm.pending)
        self.assertEqual(self.arm.command_id, 'test-command')
        self.assertEqual(self.arm.frames(20.3, True, self.binding), [encode_arm(5)])
        self.arm.sent(encode_arm(5), 20.3)
        self.ack()
        self.arm.status(1, 20.4)
        self.arm.status(0, 20.5)
        self.assertEqual(self.arm.state, 'COMPLETED')
        self.assertFalse(self.arm.pending)

    def test_reconnect_high_work_pose_is_not_duplicated_and_new_stop_cancels_replay(self):
        self.submit(cmd=3)
        self.arm.pause_link(1.1)
        self.arm.status(0, 2.)
        self.arm.resume_link(2.)
        frame, = self.arm.frames(2., True, self.binding)
        self.assertEqual(frame, encode_arm(3))
        self.arm.sent(frame, 2.)
        self.ack()
        self.arm.status(1, 2.1)
        self.arm.status(0, 2.2)
        self.assertEqual(self.arm.state, 'COMPLETED')
        self.arm.submit('next', self.binding, cmd=5)
        self.arm.sent(encode_arm(5), 3.)
        self.arm.pause_link(3.1)
        self.arm.submit('stop', self.binding, cmd=0)
        self.arm.resume_link(4.)
        self.assertEqual(self.arm.frames(4., True, self.binding), [encode_arm(0)])

    def test_reconnect_jog_only_runs_remaining_duration(self):
        self.submit(duration=1., ud_rpm=10.)
        self.arm.pause_link(1.4)
        self.arm.resume_link(5.)
        frame, = self.arm.frames(5., True, self.binding)
        self.arm.sent(frame, 5.)
        self.assertEqual(self.arm.frames(5.5, True, self.binding), [frame])
        self.assertEqual(self.arm.frames(5.7, True, self.binding), [encode_jog()])

    def test_protocol_document_examples_and_fragmented_frames(self):
        self.assertEqual(encode_arm(3).hex(), 'aa55020403000000f07d')
        frame = encode_jog(.3, 20., -10., 1)
        self.assertEqual(frame.hex(), 'aa5503109a99993e0000a041000020c101000000f0bf')
        parser = FrameParser()
        self.assertEqual(parser.feed(frame[:7]), [])
        msg, = parser.feed(frame[7:])
        self.assertEqual(msg[0], ARM_JOG)

    def test_v08_defined_commands_and_deleted_reserved_range(self):
        parser = FrameParser()
        for cmd in range(8):
            with self.subTest(cmd=cmd):
                self.assertEqual(parser.feed(encode_arm(cmd)),
                                 [(ARM_CMD, struct.pack('<B3x', cmd))])
        for cmd in (-1, *range(8, 17), True, 3.0):
            with self.subTest(cmd=cmd), self.assertRaises(ValueError):
                encode_arm(cmd)

    def test_ack_is_not_completion_and_no_repeat_frame(self):
        self.submit(cmd=3)
        self.ack()
        self.arm.status(0, 1.1)
        self.assertTrue(self.arm.pending)
        self.assertEqual(self.arm.frames(1.2, True, self.binding), [])
        self.arm.status(1, 1.3)
        self.arm.status(0, 1.4)
        self.assertEqual(self.arm.state, 'COMPLETED')
        self.assertFalse(self.arm.pending)

    def test_busy_before_ack_still_requires_fresh_idle(self):
        self.submit(cmd=3)
        self.arm.status(1, 1.1)
        self.arm.status(0, 1.2)
        self.ack()
        self.assertTrue(self.arm.pending)
        self.arm.status(0, 1.3)
        self.assertFalse(self.arm.pending)

    def test_jog_duration_measured_from_transmission_and_explicit_stop(self):
        self.arm.submit('jog', self.binding, ud_rpm=-10., duration=.4)
        self.assertIsNone(self.arm.sent_at)
        frame, = self.arm.frames(10., True, self.binding)
        self.arm.sent(frame, 10.)
        self.assertEqual(self.arm.state, 'JOGGING')
        frame, = self.arm.frames(10.3, True, self.binding)
        self.assertEqual(frame[16], 1)
        frame, = self.arm.frames(10.41, True, self.binding)
        self.assertEqual(frame, encode_jog())
        self.arm.sent(frame, 10.41)
        self.arm.status(1, 10.42)
        self.assertTrue(self.arm.pending)
        self.arm.status(0, 10.5)
        self.assertEqual(self.arm.state, 'COMPLETED')
        self.assertEqual(self.arm.frames(10.6, True, self.binding), [])

    def test_timeout_aborts_once_and_remains_unknown(self):
        self.submit(cmd=4)
        self.assertEqual(self.arm.frames(1.6, True, self.binding), [encode_arm(0)])
        self.assertEqual(self.arm.state, 'UNKNOWN')
        self.assertTrue(self.arm.pending)
        self.assertEqual(self.arm.frames(1.7, True, self.binding), [])

    def test_abort_preempts_pending_operation(self):
        self.submit(cmd=4)
        self.assertFalse(self.arm.submit('other', self.binding, cmd=3)[0])
        self.assertTrue(self.arm.submit('stop', self.binding, cmd=0)[0])
        frame, = self.arm.frames(1.2, True, self.binding)
        self.arm.sent(frame, 1.2)
        self.ack()
        self.arm.status(0, 1.3)
        self.assertEqual(self.arm.state, 'COMPLETED')

    def test_reject_permission_and_duplicate_id(self):
        self.submit(cmd=3)
        self.ack(5)
        self.assertEqual(self.arm.state, 'FAILED')
        self.assertFalse(self.arm.submit('test-command', self.binding, cmd=3)[0])
        self.arm.submit('second', self.binding, cmd=4)
        self.assertEqual(self.arm.frames(1.1, False, self.binding), [])
        self.assertEqual(self.arm.state, 'UNKNOWN')

    def test_continuous_jog_stop_then_fixed_and_no_stale_resume(self):
        self.arm.status(0, .9)
        self.assertTrue(self.arm.servo('follow',self.binding,.2,4.,True,1.,100))
        frame, = self.arm.frames(1.02,True,self.binding)
        self.assertEqual(frame,encode_jog(.2,4.,0.,1))
        self.arm.sent(frame,1.02)
        self.assertTrue(self.arm.submit('pick',self.binding,cmd=4)[0])
        frame, = self.arm.frames(1.03,True,self.binding)
        self.assertEqual(frame,encode_jog())
        self.arm.sent(frame,1.03)
        self.assertEqual(self.arm.frames(1.04,True,self.binding),[])
        self.arm.status(0,1.05)
        frame, = self.arm.frames(1.06,True,self.binding)
        self.assertEqual(frame,encode_arm(4))
        self.arm.sent(frame,1.06)
        self.ack()
        self.arm.status(1,1.07)
        self.arm.status(0,1.2)
        self.assertFalse(self.arm.servo('follow',self.binding,.2,4.,True,1.21,200))
        self.assertEqual(self.arm.frames(1.22,True,self.binding),[])
        self.assertTrue(self.arm.servo('next',self.binding,-.1,3.,True,1.23,201))

    def test_continuous_timeout_and_permission_loss_explicitly_stop(self):
        for allowed in (True,False):
            arm=ArmController()
            arm.status(0,.9)
            arm.servo('follow',self.binding,.2,4.,True,1.,100)
            self.assertEqual(arm.frames(1.2,allowed,self.binding),[encode_jog()])
            self.assertFalse(arm.servo('follow',self.binding,.2,4.,True,1.21,101))

    def test_short_loss_recovery_waits_for_new_idle_before_restart(self):
        self.arm.status(0,.9)
        self.arm.servo('one',self.binding,.2,4.,True,1.,100)
        self.arm.servo('one',self.binding,0.,0.,False,1.01,101)
        frame,=self.arm.frames(1.02,True,self.binding)
        self.arm.sent(frame,1.02)
        self.assertTrue(self.arm.servo('two',self.binding,.1,2.,True,1.03,102))
        self.assertEqual(self.arm.frames(1.04,True,self.binding),[])
        self.arm.status(0,1.05)
        self.assertEqual(self.arm.frames(1.06,True,self.binding),[encode_jog(.1,2.,0.,1)])

    def test_abort_during_servo_sends_stop_then_abort_without_waiting_idle(self):
        self.arm.status(0,.9)
        self.arm.servo('stream',self.binding,.2,3.,True,1.,100)
        self.arm.submit('abort',self.binding,cmd=0)
        frame,=self.arm.frames(1.01,True,self.binding)
        self.assertEqual(frame,encode_jog())
        self.arm.sent(frame,1.01)
        frame,=self.arm.frames(1.02,True,self.binding)
        self.assertEqual(frame,encode_arm(0))
        self.arm.sent(frame,1.02)
        self.ack()
        self.arm.status(0,1.03)
        self.assertEqual(self.arm.state,'COMPLETED')


if __name__ == '__main__':
    unittest.main()
