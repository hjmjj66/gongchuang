"""Preserve DDS sample metadata on ROS2 Humble.

Humble's stock executor explicitly discards take_message()[1]. This narrowly
scoped override keeps it only for the final chassis subscription. All other
subscriptions, timers and services use the standard executor unchanged.
"""

from rclpy.executors import SingleThreadedExecutor


class CommandExecutor(SingleThreadedExecutor):
    def __init__(self, command_subscription, **kwargs):
        super().__init__(**kwargs)
        self._command_subscription = command_subscription

    def _take_subscription(self, sub):
        if sub is not self._command_subscription:
            return super()._take_subscription(sub)
        with sub.handle:
            sample = sub.handle.take_message(sub.msg_type, sub.raw)
        # The stock _execute_subscription hands this pair to our callback.
        return sample
