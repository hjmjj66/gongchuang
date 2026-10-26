"""Protocol and motion-inhibition regressions; no hardware or ROS required."""

import math
import struct
import unittest

from agv_cboard.chassis_adapter import ChassisAdapter
from agv_cboard.protocol import (
    CHASSIS_CMD, CHASSIS_STRUCT, ODOM, ODOM_STRUCT, STATUS, STATUS_STRUCT,
    FrameParser, crc16, encode_chassis, encode_frame,
)


def status(t=1, source=2, chassis=1, online=15, errors=0):
    return STATUS_STRUCT.pack(t, chassis, 0, online, errors, 0., 0., 0., source)


def odom(t=1, x=0., y=0., yaw=0., flags=0x0D):
    return ODOM_STRUCT.pack(t, x, y, yaw, 0., 0., 0., flags)


ZERO = encode_chassis(0., 0., 0., 0)


class ProtocolTests(unittest.TestCase):
    EXAMPLES = (
        'AA 55 01 10 CD CC 4C 3E 00 00 00 00 00 00 00 3F 01 00 00 00 7A 46',
        'AA 55 81 20 40 E2 01 00 00 00 80 3F 00 00 00 3F CD CC CC 3D CD CC 4C 3E 00 00 00 00 00 00 00 3F 01 00 00 00 17 BE',
        'AA 55 82 18 40 E2 01 00 01 00 0F 00 00 00 C8 42 00 00 48 42 00 00 00 00 02 00 00 00 EC BD',
        'AA 55 83 04 02 00 00 00 EF 90',
    )

    def test_official_examples_and_crc_check(self):
        self.assertEqual(crc16(b'123456789'), 0x4B37)
        for text in self.EXAMPLES:
            frame = bytes.fromhex(text)
            self.assertEqual(encode_frame(frame[2], frame[4:-2]), frame)
        self.assertEqual(encode_chassis(.2, 0., .5, 1), bytes.fromhex(self.EXAMPLES[0]))

    def test_every_split_and_byte_at_a_time(self):
        for text in self.EXAMPLES:
            frame = bytes.fromhex(text)
            expected = [(frame[2], frame[4:-2])]
            for split in range(len(frame) + 1):
                parser = FrameParser()
                self.assertEqual(parser.feed(frame[:split], 1.) +
                                 parser.feed(frame[split:], 1.001), expected)
            parser = FrameParser()
            result = []
            for byte in frame:
                result.extend(parser.feed(bytes((byte,)), 1.))
            self.assertEqual(result, expected)

    def test_noise_bad_crc_glued_and_unknown_frames(self):
        good = encode_frame(STATUS, status())
        bad = bytearray(good)
        bad[8] ^= 1
        unknown = encode_frame(0x99, b'\xaa\x55')
        parser = FrameParser()
        self.assertEqual(parser.feed(b'noise\xaa' + bad + unknown + good),
                         [(0x99, b'\xaa\x55'), (STATUS, status())])

    def test_false_length_expires_and_preserves_later_frame(self):
        parser = FrameParser(partial_timeout=.1)
        good = encode_frame(STATUS, status())
        self.assertEqual(parser.feed(b'\xaa\x55\x81\xff' + good, 0.), [])
        self.assertEqual(parser.feed(b'', .101), [(STATUS, status())])

    def test_reset_and_memory_bound(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b'x' * 10000 + b'\xaa'), [])
        parser.reset()
        self.assertEqual(parser.feed(b'\x55'), [])
        self.assertEqual(parser.feed(encode_frame(ODOM, odom())), [(ODOM, odom())])

    def test_large_valid_burst_and_pending_prefix_are_lossless(self):
        frames = [(ODOM, odom(t)) for t in range(1000)]
        stream = b''.join(encode_frame(mid, payload) for mid, payload in frames)
        parser = FrameParser()
        self.assertEqual(parser.feed(stream[:3], 1.), [])
        self.assertEqual(parser.feed(stream[3:], 1.01), frames)
        self.assertLessEqual(len(parser._buffer), 260)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.a = ChassisAdapter()
        self.a.on_connected(10.)
        self.ready()

    def ready(self, t=10., tick=1, source=2, flags=0x0D, x=0., y=0., yaw=0.):
        self.a.on_frame(STATUS, status(tick, source), t)
        self.a.on_frame(ODOM, odom(tick, x, y, yaw, flags), t)

    def test_axes_units_and_50ms_watchdog(self):
        self.assertTrue(self.a.on_command(.2, .3, 90., 10.01))
        self.assertEqual(self.a.command_frame(10.059), encode_chassis(.2, .3, math.pi/2, 1))
        self.assertEqual(self.a.command_frame(10.061), ZERO)
        self.assertEqual(self.a.command_reason, 'input timed out')

    def test_single_axis_commands_are_body_fixed_at_every_heading(self):
        # Changing the odometry origin/heading must not rotate target velocities
        # or introduce another axis in the transmitted command.
        for yaw in (0., math.pi/4, math.pi/2, -math.pi, 4*math.pi):
            for command in ((.05, 0., 0.), (-.05, 0., 0.),
                            (0., .05, 0.), (0., -.05, 0.),
                            (0., 0., 10.), (0., 0., -10.)):
                with self.subTest(yaw=yaw, command=command):
                    adapter = ChassisAdapter()
                    adapter.on_connected(10.)
                    adapter.on_frame(STATUS, status(), 10.)
                    adapter.on_frame(ODOM, odom(x=2., y=3., yaw=yaw), 10.)
                    adapter.on_frame(ODOM, odom(11, x=2., y=3., yaw=yaw+.3), 10.01)
                    self.assertTrue(adapter.on_command(*command, 10.02))
                    frame = adapter.command_frame(10.021)
                    self.assertEqual(frame[:4], bytes.fromhex('AA 55 01 10'))
                    self.assertEqual(len(frame), 22)
                    vx, vy, wz, mode = struct.unpack_from('<fffB', frame, 4)
                    self.assertAlmostEqual(vx, command[0], places=7)
                    self.assertAlmostEqual(vy, command[1], places=7)
                    self.assertAlmostEqual(wz, math.radians(command[2]), places=7)
                    self.assertEqual(mode, 1)
                    self.assertEqual(frame[17:20], b'\x00\x00\x00')

    def test_no_input_and_zero_input(self):
        self.assertEqual(self.a.command_frame(10.01), ZERO)
        self.a.on_command(.2, 0., 0., 10.02)
        self.assertTrue(self.a.on_command(0., 0., 0., 10.03))
        self.assertEqual(self.a.command_frame(10.04), ZERO)
        self.assertTrue(self.a.command_fresh(10.04))

    def test_source_age_order_and_nonfinite(self):
        for age in (.051, -0.001, float('inf')):
            self.assertFalse(self.a.on_command(1., 0., 0., 10.01, age))
        self.assertTrue(self.a.on_command(1., 0., 0., 10.02, source_timestamp_ns=20))
        self.assertFalse(self.a.on_command(1., 0., 0., 10.03, source_timestamp_ns=19))
        self.assertFalse(self.a.on_command(float('nan'), 0., 0., 10.04))
        self.assertEqual(self.a.command_frame(10.05), ZERO)

    def test_rc_observation_pc_reset_automatic_recovery(self):
        session, epoch = self.a.odom_session_id, self.a.motion_epoch
        self.ready(10.02, 21, source=1, x=1., y=2.)
        self.assertEqual(self.a.local_odom().x, 1.)
        self.assertEqual(self.a.command_frame(10.021), ZERO)
        self.assertIn('not PC', self.a.health_reason(10.021))
        self.assertNotEqual(self.a.motion_epoch, epoch)
        self.ready(10.04, 41, x=5., y=7., yaw=math.pi/2)
        self.assertNotEqual(self.a.odom_session_id, session)
        self.assertTrue(self.a.update(10.041))
        self.assertEqual((self.a.local_odom().x,self.a.local_odom().y,self.a.local_odom().yaw),(0.,0.,0.))
        self.assertTrue(self.a.on_command(0., .1, 0., 10.05))
        current = self.a.odom_session_id
        self.ready(10.06, 61, x=6., y=7., yaw=math.pi/2)
        self.assertEqual(self.a.odom_session_id, current)
        self.assertAlmostEqual(self.a.local_odom().y,-1.)

    def test_local_origin_rotation_uses_forward_x_left_y(self):
        for yaw, forward_delta, left_delta in (
                (0., (1., 0.), (0., 1.)),
                (math.pi/2, (0., 1.), (-1., 0.)),
                (math.pi, (-1., 0.), (0., -1.)),
                (-math.pi/2, (0., -1.), (1., 0.))):
            for delta, expected in ((forward_delta, (1., 0.)), (left_delta, (0., 1.))):
                with self.subTest(yaw=yaw, delta=delta):
                    self.a = ChassisAdapter()
                    self.a.on_connected(10.)
                    self.ready(x=2., y=3., yaw=yaw)
                    self.ready(10.02, 21, x=2.+delta[0], y=3.+delta[1], yaw=yaw+.3)
                    local = self.a.local_odom()
                    # The protocol carries float32, including the origin yaw.
                    self.assertAlmostEqual(local.x, expected[0], delta=1e-6)
                    self.assertAlmostEqual(local.y, expected[1], delta=1e-6)
                    self.assertAlmostEqual(local.yaw, .3, delta=1e-6)

    def test_origin_uses_post_switch_sample(self):
        self.ready(10.02, 21, source=1, x=9.)
        self.a.on_frame(STATUS,status(41,2),10.04)
        self.assertFalse(self.a.origin_initialized)
        self.assertIsNone(self.a.local_odom())
        self.a.on_frame(ODOM,odom(42,x=10.),10.041)
        self.assertEqual(self.a.local_odom().x,0.)

    def test_health_recovery_preserves_origin_rejects_old_binding(self):
        session, epoch = self.a.odom_session_id,self.a.motion_epoch
        self.ready(10.02,21,flags=0x0F)
        self.assertFalse(self.a.update(10.021))
        self.ready(10.04,41,x=1.)
        self.assertTrue(self.a.update(10.041))
        self.assertEqual(self.a.odom_session_id,session)
        self.assertEqual(self.a.local_odom().x,1.)
        self.assertFalse(self.a.on_command(.1,0.,0.,10.05,odom_session_id=session,motion_epoch=epoch))
        self.assertTrue(self.a.on_command(.1,0.,0.,10.06,odom_session_id=session,motion_epoch=self.a.motion_epoch))

    def test_slip_warning_policy_preserves_origin_epoch_and_bound_command(self):
        self.a = ChassisAdapter(stop_on_slip=False)
        self.a.on_connected(10.)
        self.ready()
        session, epoch = self.a.odom_session_id, self.a.motion_epoch
        self.assertTrue(self.a.on_command(0., .8, 0., 10.01,
                                        odom_session_id=session, motion_epoch=epoch))
        self.ready(10.02, 21, flags=0x0F, y=.01)
        self.assertTrue(self.a.update(10.021))
        self.assertEqual(self.a.health_reason(10.021), '')
        self.assertEqual((self.a.odom_session_id, self.a.motion_epoch), (session, epoch))
        self.assertEqual(self.a.command_frame(10.021), encode_chassis(0., .8, 0., 1))
        self.assertEqual(self.a.odom.flags, 0x0F)
        self.assertTrue(self.a.odom_flags_ok(self.a.odom.flags))
        self.ready(10.03, 31, flags=0x0D, y=.02)
        self.assertEqual((self.a.odom_session_id, self.a.motion_epoch), (session, epoch))
        # No speed threshold is introduced by the slip policy.
        for index, speed in enumerate((.8, 2., 2.5)):
            now = 10.04 + index*.01
            self.assertTrue(self.a.on_command(0., speed, 0., now,
                                            odom_session_id=session, motion_epoch=epoch))
            self.assertEqual(self.a.command_frame(now), encode_chassis(0., speed, 0., 1))

    def test_slip_warning_policy_can_establish_origin(self):
        self.a = ChassisAdapter(stop_on_slip=False)
        self.a.on_connected(10.)
        self.ready(flags=0x0F, x=5., y=7.)
        self.assertTrue(self.a.origin_initialized)
        self.assertTrue(self.a.update(10.01))
        self.assertEqual((self.a.local_odom().x, self.a.local_odom().y), (0., 0.))

    def test_slip_warning_policy_keeps_other_odom_faults(self):
        for missing_bit in (0x01, 0x04, 0x08):
            with self.subTest(missing_bit=missing_bit):
                self.a = ChassisAdapter(stop_on_slip=False)
                self.a.on_connected(10.)
                self.ready()
                epoch = self.a.motion_epoch
                self.a.on_command(0., .8, 0., 10.01)
                self.ready(10.02, 21, flags=0x0F & ~missing_bit)
                self.assertFalse(self.a.update(10.021))
                self.assertFalse(self.a.odom_flags_ok(self.a.odom.flags))
                self.assertEqual(self.a.command_frame(10.021), ZERO)
                self.assertGreater(self.a.motion_epoch, epoch)

    def test_slip_warning_policy_keeps_status_faults_and_timeout(self):
        for fault in ({'source': 1}, {'online': 14}, {'errors': 1},
                      {'errors': 2}, {'errors': 16}):
            with self.subTest(fault=fault):
                self.a = ChassisAdapter(stop_on_slip=False)
                self.a.on_connected(10.)
                self.ready(flags=0x0F)
                self.a.on_command(0., .8, 0., 10.01)
                self.a.on_frame(STATUS, status(21, **fault), 10.02)
                self.assertFalse(self.a.update(10.021))
                self.assertEqual(self.a.command_frame(10.021), ZERO)
        self.a = ChassisAdapter(stop_on_slip=False)
        self.a.on_connected(10.)
        self.ready(flags=0x0F)
        self.assertFalse(self.a.update(10.201))
        self.assertEqual(self.a.command_frame(10.201), ZERO)

    def test_timeout_cannot_hide_between_frames(self):
        epoch=self.a.motion_epoch
        self.ready(10.25,251)
        self.assertTrue(self.a.update(10.251))
        self.assertNotEqual(self.a.motion_epoch,epoch)
        self.assertFalse(self.a.on_command(.1,0.,0.,10.252,source_age=.01))
        self.assertTrue(self.a.on_command(.1,0.,0.,10.253))

    def test_reconnect_and_process_restart_new_session(self):
        old=self.a.odom_session_id
        self.a.on_disconnected(10.02)
        self.a.on_connected(10.03)
        self.ready(10.04,41,x=8.)
        self.assertNotEqual(self.a.odom_session_id,old)
        self.assertEqual(self.a.local_odom().x,0.)
        self.assertNotEqual(ChassisAdapter().odom_session_id,self.a.odom_session_id)

    def test_restart_two_streams_once_then_recover(self):
        self.a.on_connected(10.005)
        self.ready(10.01,10001,x=1.)
        old=self.a.odom_session_id
        self.assertFalse(self.a.on_frame(ODOM,odom(2,x=4.),10.02))
        self.assertFalse(self.a.update(10.021))
        self.a.on_frame(STATUS,status(3),10.03)
        new=self.a.odom_session_id
        self.assertNotEqual(new,old)
        self.assertEqual(self.a.local_odom().x,0.)
        self.ready(10.04,13,x=4.1)
        self.assertEqual(self.a.odom_session_id,new)
        self.assertAlmostEqual(self.a.local_odom().x,.1,places=5)
        self.assertTrue(self.a.update(10.041))
        self.assertFalse(self.a.on_frame(ODOM,odom(10005),10.042))

    def test_isolated_reordered_frame_does_not_reset(self):
        self.ready(10.01,101)
        old=self.a.odom_session_id
        self.assertFalse(self.a.on_frame(ODOM,odom(90),10.02))
        self.ready(10.03,121)
        self.assertTrue(self.a.update(10.031))
        self.assertEqual(self.a.odom_session_id,old)

    def test_natural_tick_wrap_and_duplicates(self):
        self.a.on_connected(10.)
        self.ready(10.,0xFFFFFFF0)
        old=self.a.odom_session_id
        self.ready(10.02,4,x=.1)
        sequence=self.a.odom_sequence
        self.ready(10.021,4,x=.1)
        self.assertEqual(self.a.odom_sequence,sequence)
        self.assertEqual(self.a.odom_session_id,old)

    def test_malformed_recovers_and_finite_output(self):
        self.a.on_frame(ODOM,b'bad',10.02)
        self.assertFalse(self.a.update(10.021))
        self.ready(10.04,41)
        self.assertTrue(self.a.update(10.041))
        self.assertTrue(self.a.reset_fault(10.042)[0])

    def test_continuous_yaw_not_compensated(self):
        self.ready(10.02,21,yaw=-7.)
        self.assertAlmostEqual(self.a.local_odom().yaw,-7.)

    def test_yaw_payload_offset_and_status_axis_are_independent(self):
        # Build by documented byte offsets, independently of ODOM_STRUCT.
        payload = bytearray(32)
        struct.pack_into('<I', payload, 0, 21)
        for offset, value in ((4, 2.), (8, -3.), (12, -7.25),
                              (16, .125), (20, -.25), (24, .5)):
            struct.pack_into('<f', payload, offset, value)
        payload[28] = 0x0D
        self.assertTrue(self.a.on_frame(ODOM, payload, 10.02))
        self.assertEqual(self.a.odom.yaw, -7.25)
        self.assertEqual(self.a.local_odom().yaw, -7.25)
        mechanism_status = bytearray(status(31))
        struct.pack_into('<f', mechanism_status, 16, 100.)
        self.a.on_frame(STATUS, mechanism_status, 10.03)
        self.assertEqual(self.a.status.yaw_rad, 100.)
        self.assertEqual(self.a.local_odom().yaw, -7.25)

    def test_fixed_yaw_never_integrates_feedback_or_commands(self):
        for step in range(1, 101):
            now = 10. + step*.01
            self.a.on_frame(STATUS, status(1+step*10), now)
            payload = ODOM_STRUCT.pack(1+step*10, step*.01, -step*.02,
                                       0., .2, -.3, -.5, 0x0D)
            self.a.on_frame(ODOM, payload, now)
            self.a.on_command(.2, -.3, -30., now+.001)
            self.a.command_frame(now+.002)
            self.assertEqual(self.a.local_odom().yaw, 0.)
        for now in (11.1, 11.3, 12., 20.):
            self.a.update(now)
            self.a.command_frame(now)
            self.assertEqual(self.a.local_odom().yaw, 0.)


if __name__ == '__main__':
    unittest.main()
