"""Exercise real ROS message validation without starting ROS or any hardware."""

from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message

from agv_perception.arm_servo import ArmServoPublisher
from agv_perception.work_control import PID


class ArmServoTests(unittest.TestCase):
    def setUp(self):
        self.messages = []
        node = SimpleNamespace(
            create_publisher=lambda *args: SimpleNamespace(publish=self.messages.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
        self.publisher = ArmServoPublisher(node)
        self.task = SimpleNamespace(servo_enabled=True, finished=False, arm_velocity=(0, 0))

    def test_integer_pid_limits_serialize_for_both_directions(self):
        gains = dict(kp=1, ki=0, kd=0)
        for error in (-100., 100.):
            with self.subTest(error=error):
                yaw = PID(gains, 1).update(error, 1.)
                fb = PID(gains, 2).update(error, 1.)
                self.assertIs(type(yaw), int)
                self.assertIs(type(fb), int)
                self.task.arm_velocity = (yaw, fb)
                self.publisher.publish(self.task, ('session', 1))
                message = self.messages[-1]
                self.assertTrue(message.enabled)
                self.assertIs(type(message.yaw_wz), float)
                self.assertIs(type(message.fb_rpm), float)
                self.assertEqual((message.yaw_wz, message.fb_rpm), (yaw, fb))
                self.assertTrue(serialize_message(message))

    def test_zero_hold_and_stop_still_serialize_and_close_the_stream(self):
        self.task.arm_velocity = (1, -2)
        self.publisher.publish(self.task, ('session', 1))
        stream = self.messages[-1].stream_id
        self.task.servo_enabled = False
        self.publisher.publish(self.task, ('session', 1), hold_zero=True)
        held = self.messages[-1]
        self.assertTrue(held.enabled)
        self.assertEqual(held.stream_id, stream)
        self.assertEqual((held.yaw_wz, held.fb_rpm), (0., 0.))
        self.assertTrue(serialize_message(held))
        self.publisher.publish(None, ('session', 1))
        stopped = self.messages[-1]
        self.assertFalse(stopped.enabled)
        self.assertEqual(stopped.stream_id, stream)
        self.assertEqual((stopped.yaw_wz, stopped.fb_rpm), (0., 0.))
        self.assertTrue(serialize_message(stopped))
        self.assertEqual(self.publisher.stream_id, '')
        self.publisher.publish(None, ('session', 1))
        self.assertEqual(len(self.messages), 3)
