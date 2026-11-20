"""Single outstanding arm operation on the existing CDC connection."""

import math

from .protocol import ACK, ACK_STRUCT, ARM_CMD, ARM_JOG, encode_arm, encode_jog


class ArmController:
    def __init__(self, ack_timeout=2., action_timeout=30.):
        if not all(math.isfinite(t) and t > 0 for t in (ack_timeout, action_timeout)):
            raise ValueError('arm timeouts must be finite and positive')
        self.ack_timeout, self.action_timeout = ack_timeout, action_timeout
        self.command_id = ''
        self.state, self.reason = 'IDLE', ''
        self.pending = False
        self.idle = False
        self.binding = None
        self.cmd = None
        self.sent_at = self.stop_sent_at = None
        self._accepted = self._busy_seen = False
        self._stop_frame = None
        self._seen_ids = set()
        self.servo_id = ''
        self.servo_frame = None
        self.servo_at = None
        self.servo_binding = None
        self.servo_stopping = False
        self.servo_stop_sent = None
        self._closed_streams = set()
        self._servo_stamp = 0
        self.servo_timeout = .1
        self._link_paused_at = None
        self._replay = None
        self._recover_wait_idle = False

    @property
    def servo_busy(self):
        return self.servo_frame is not None or self.servo_stopping

    def servo(self, stream_id, binding, yaw_wz, fb_rpm, enabled, now, stamp):
        if (not stream_id or stream_id in self._closed_streams or stamp <= self._servo_stamp or
                not all(math.isfinite(v) for v in (yaw_wz, fb_rpm))):
            return False
        self._servo_stamp = stamp
        if not enabled:
            self._closed_streams.add(stream_id)
            if stream_id == self.servo_id:
                self.stop_servo()
            return True
        if self.state == 'UNKNOWN' or self.pending or (self.servo_id and stream_id != self.servo_id):
            self._closed_streams.add(stream_id)
            return False
        if self.servo_frame is None and not self.idle and not self.servo_stopping:
            return False
        self.servo_id, self.servo_binding, self.servo_at = stream_id, binding, now
        self.servo_frame = encode_jog(yaw_wz=yaw_wz, fb_rpm=fb_rpm, mode=1)
        self.state, self.reason = 'SERVO', ''
        return True

    def stop_servo(self):
        if self.servo_frame is not None:
            self._closed_streams.add(self.servo_id)
            self.servo_frame = None
            self.servo_stopping = True
            self.servo_stop_sent = None
            self._stop_frame = encode_jog()
        self.servo_id = ''

    def submit(self, command_id, binding, *, cmd=None, ud_rpm=0., duration=0.):
        if not command_id or command_id in self._seen_ids:
            return False, 'empty or previously used command_id'
        if self.pending and cmd != 0:
            return False, 'arm operation pending; no queue'
        interrupt_jog = self.pending and self.cmd is None and cmd == 0
        if cmd is not None:
            frame = encode_arm(cmd)
        else:
            if not math.isfinite(duration) or not 0 < duration <= self.action_timeout:
                return False, 'invalid jog duration'
            frame = encode_jog(ud_rpm=ud_rpm, mode=1)
        self.stop_servo()
        self._replay = None
        self._recover_wait_idle = False
        self._seen_ids.add(command_id)
        self.command_id, self.binding = command_id, binding
        self.cmd, self.duration, self.frame = cmd, duration, frame
        self.state, self.reason, self.pending = 'QUEUED', '', True
        self.sent_at = self.stop_sent_at = None
        self._accepted = self._busy_seen = False
        if interrupt_jog:
            self._stop_frame = encode_jog()
        return True, ''

    def pause_link(self, now):
        if self._link_paused_at is not None:
            return
        self._link_paused_at = now
        self.stop_servo()
        if self.pending and self.sent_at is not None:
            if self.cmd is not None and self.cmd != 0:
                # ACK/completion may have been lost; recover via high work pose.
                if self._replay is None:
                    self._replay = (self.cmd, self.frame, self.duration)
                self._recover_wait_idle = True
            elif self.cmd is None:
                self.duration = max(0., self.duration-(now-self.sent_at))
                self.state, self.sent_at = 'QUEUED', None

    def resume_link(self, now):
        if self._link_paused_at is None:
            return
        duration = now-self._link_paused_at
        if self.sent_at is not None:
            self.sent_at += duration
        if self.stop_sent_at is not None:
            self.stop_sent_at += duration
        if self.servo_stopping:
            self.servo_stop_sent = None
            self._stop_frame = encode_jog()
        self._link_paused_at = None

    def frames(self, now, allowed, binding):
        if self._link_paused_at is not None:
            return []
        if self._recover_wait_idle:
            if binding != self.binding:
                self.invalidate('arm coordinate session changed during reconnection')
                return []
            if self.sent_at is not None and now-self.sent_at > self.action_timeout:
                self.invalidate('arm recovery waiting for idle timed out')
                return []
            if not allowed or not self.idle:
                return []
            self.cmd, self.frame = 3, encode_arm(3)
            self.state, self.sent_at = 'QUEUED', None
            self._accepted = self._busy_seen = False
            self._recover_wait_idle = False

        if self.servo_frame is not None and (not allowed or binding != self.servo_binding or
                                              now-self.servo_at > self.servo_timeout):
            self.invalidate('continuous JOG input stale or permission lost')
        if not allowed or (self.pending and binding != self.binding):
            if self.pending:
                self.invalidate('arm control permission/coordinate session lost')
            if self._stop_frame is not None:
                frame, self._stop_frame = self._stop_frame, None
                return [frame]
            return []
        if self._stop_frame is not None:
            frame, self._stop_frame = self._stop_frame, None
            return [frame]
        if self.servo_stopping and not (self.pending and self.cmd == 0 and self.state == 'QUEUED'):
            if self.servo_stop_sent is not None and now-self.servo_stop_sent > self.ack_timeout:
                self.invalidate('continuous JOG stop not confirmed idle')
            return []
        if self.servo_frame is not None:
            return [self.servo_frame]
        if not self.pending or self.state == 'UNKNOWN':
            return []
        if self.sent_at is not None:
            limit = self.ack_timeout if self.state == 'WAIT_ACK' else self.action_timeout
            if now - self.sent_at > limit:
                self.invalidate('arm ACK/completion timeout; not retried')
                self._stop_frame = encode_jog() if self.cmd is None else encode_arm(0)
                return self.frames(now, allowed, binding)
        if self.state == 'QUEUED':
            return [self.frame]
        if self.state == 'JOGGING':
            return [encode_jog() if now-self.sent_at >= self.duration else self.frame]
        return []

    def sent(self, frame, now):
        if self.servo_stopping and frame == encode_jog():
            self.servo_stop_sent = now
            return
        if not self.pending or self.state == 'UNKNOWN':
            return
        if frame[2] == ARM_CMD and self.state == 'QUEUED':
            self.sent_at, self.state = now, 'WAIT_ACK'
        elif frame[2] == ARM_JOG and self.cmd is None:
            if self.state == 'QUEUED':
                self.sent_at, self.state = now, 'JOGGING'
            elif self.state == 'JOGGING' and frame[16] == 0:
                self.stop_sent_at, self.state = now, 'STOPPING'

    def ack(self, msg_id, payload):
        if msg_id != ACK or not self.pending or self.state not in ('WAIT_ACK', 'MOVING'):
            return
        if len(payload) != ACK_STRUCT.size:
            self.invalidate('malformed arm ACK')
            return
        acked, result, _ = ACK_STRUCT.unpack(payload)
        if acked != ARM_CMD:
            return
        if result:
            self.state, self.reason, self.pending = 'FAILED', f'ARM_CMD rejected: {result}', False
        else:
            self._accepted, self.state = True, 'MOVING'

    def status(self, arm_state, now):
        self.idle = arm_state == 0
        if (self.servo_stopping and self.servo_stop_sent is not None and
                self.idle and now > self.servo_stop_sent):
            self.servo_stopping = False
            self.servo_stop_sent = None
        if self._recover_wait_idle or self._link_paused_at is not None:
            return
        if not self.pending or self.sent_at is None:
            return
        if arm_state == 3:
            self.invalidate('arm reports BLOCKED/error')
        elif self.state == 'STOPPING' and self.idle and now > self.stop_sent_at:
            self.state, self.pending = 'COMPLETED', False
        elif self.cmd is not None and self.state in ('WAIT_ACK', 'MOVING'):
            self._busy_seen |= arm_state in (1, 2)
            if self._accepted and self.idle and (self._busy_seen or self.cmd == 0):
                self.state, self.pending = 'COMPLETED', False
                if self._replay is not None:
                    cmd, frame, duration = self._replay
                    self._replay = None
                    if cmd != 3:
                        self.cmd, self.frame, self.duration = cmd, frame, duration
                        self.state, self.sent_at, self.pending = 'QUEUED', None, True
                        self._accepted = self._busy_seen = False

    def invalidate(self, reason):
        self._replay = None
        self._recover_wait_idle = False
        self._link_paused_at = None
        active = self.pending or self.servo_busy
        self.stop_servo()
        if active:
            self.state, self.reason = 'UNKNOWN', reason
        self.idle = False

