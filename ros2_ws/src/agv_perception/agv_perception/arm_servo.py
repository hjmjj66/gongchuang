"""Fresh continuous-arm messages shared by the production and browser hosts."""

import uuid
from agv_interfaces.msg import ArmServoCommand


class ArmServoPublisher:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(ArmServoCommand, '/C/arm_servo', 1)
        self.stream_id = ''

    def publish(self, task, binding, *, hold_zero=False):
        enabled = task is not None and (task.servo_enabled or hold_zero) and not task.finished
        if not enabled and not self.stream_id:
            return
        if enabled and not self.stream_id:
            self.stream_id = uuid.uuid4().hex
        yaw, fb = task.arm_velocity if enabled and task.servo_enabled else (0., 0.)
        message = ArmServoCommand(stream_id=self.stream_id, enabled=enabled,
                                  yaw_wz=float(yaw), fb_rpm=float(fb),
                                  odom_session_id=binding[0], motion_epoch=binding[1])
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(message)
        if not enabled:
            self.stream_id = ''
