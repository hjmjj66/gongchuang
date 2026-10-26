"""Pure CDC state machine. Board/body x FORWARD, y LEFT, yaw CCW.

Observation, coordinate sessions and motion permission are independent. The
caller serializes access and supplies monotonic receive times.
"""
from dataclasses import dataclass, replace
import math
import uuid

from .protocol import ODOM, STATUS, encode_chassis, unpack_odom, unpack_status


@dataclass(frozen=True)
class Status:
    t_ms: int
    chassis: int
    arm: int
    online: int
    errors: int
    fb_mm: float
    ud_mm: float
    yaw_rad: float
    source: int


@dataclass(frozen=True)
class Odom:
    t_ms: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    flags: int


class ChassisAdapter:
    def __init__(self, command_timeout=0.05, status_timeout=0.2, odom_timeout=0.2,
                 stop_on_slip=True):
        self.command_timeout = command_timeout
        self.status_timeout = status_timeout
        self.odom_timeout = odom_timeout
        self.stop_on_slip = stop_on_slip
        self.connected = False
        self.status = self.odom = None
        self.status_received = self.odom_received = None
        self.odom_session_id = uuid.uuid4().hex
        self.motion_epoch = 0
        self.odom_sequence = 0
        self._origin = None
        self._origin_after = float('-inf')
        self._ready = False
        self._command = self._command_received = None
        self._input_time = None
        self.command_reason = 'no input'
        self._command_gate = float('-inf')
        self._last_command_time = float('-inf')
        self._last_source_timestamp_ns = None
        self._malformed = set()
        self._restart_candidates = {}

    @property
    def origin_initialized(self):
        return self._origin is not None

    def clear_command(self):
        self._command = self._command_received = None

    def _inhibit(self, now):
        if self._ready:
            self.motion_epoch += 1
        self._ready = False
        self._command_gate = now
        self._input_time = None
        self.clear_command()

    def _new_origin(self, now):
        self._inhibit(now)
        self.motion_epoch += 1
        self.odom_session_id = uuid.uuid4().hex
        self._origin = None
        self._origin_after = now

    def on_connected(self, now):
        self._new_origin(now)
        self.connected = True
        self.status = self.odom = None
        self.status_received = self.odom_received = None
        self._last_command_time = float('-inf')
        self._last_source_timestamp_ns = None
        self._malformed.clear()
        self._restart_candidates.clear()

    def on_disconnected(self, now, reason='serial disconnected'):
        del reason
        self.connected = False
        self._inhibit(now)
        self.status_received = self.odom_received = None

    def status_fresh(self, now):
        return (self.connected and self.status_received is not None
                and 0 <= now - self.status_received <= self.status_timeout)

    def odom_fresh(self, now):
        return (self.connected and self.odom_received is not None
                and 0 <= now - self.odom_received <= self.odom_timeout)

    def odom_flags_ok(self, flags):
        # Keep valid/IMU/encoder requirements independent of the slip policy.
        # The board's original flags remain visible in every feedback topic.
        return flags & 0x0D == 0x0D and not (self.stop_on_slip and flags & 0x02)

    def health_reason(self, now):
        if not self.connected:
            return 'serial disconnected'
        if self._restart_candidates:
            return 'board clock rollback; awaiting restart confirmation'
        if self._malformed:
            return 'malformed STATUS/ODOM; awaiting valid replacement'
        if not self.status_fresh(now):
            return 'STATUS missing or timed out'
        if self.status.source != 2:
            return 'control source is not PC'
        if self.status.chassis not in (0, 1, 2):
            return 'wheel offline or unknown chassis state'
        if self.status.online & 0x0F != 0x0F:
            return 'one or more wheels offline'
        if self.status.errors & 0x13:
            return 'chassis emergency stop/CAN/IMU fault'
        if not self.odom_fresh(now):
            return 'ODOM missing or timed out'
        if not self.odom_flags_ok(self.odom.flags):
            return f'ODOM invalid/slip/IMU/encoder fault (status=0x{self.odom.flags:02X})'
        if not self.origin_initialized:
            return 'awaiting new valid ODOM to establish origin'
        return ''

    def update(self, now):
        reason = self.health_reason(now)
        if reason:
            if self._ready:
                self._inhibit(now)
            self.clear_command()
        elif not self._ready:
            self._ready = True
            self._command_gate = now
            self.clear_command()
        return self._ready

    def command_age(self, now):
        return -1.0 if self._input_time is None else max(0.0, now - self._input_time)

    def command_fresh(self, now):
        return self._input_time is not None and 0 <= now-self._input_time <= self.command_timeout

    def on_command(self, vx, vy, omega_deg_s, now, source_age=0.0,
                   source_timestamp_ns=None, odom_session_id='', motion_epoch=0):
        ready = self.update(now)
        if not math.isfinite(source_age) or not 0 <= source_age <= self.command_timeout:
            self.command_reason = 'input source time missing/future/expired'
            return False
        sample_time = now-source_age
        if source_timestamp_ns is not None:
            if (source_timestamp_ns <= 0 or
                    (self._last_source_timestamp_ns is not None and
                     source_timestamp_ns <= self._last_source_timestamp_ns)):
                self.command_reason = 'duplicate or reordered input'
                return False
            self._last_source_timestamp_ns = source_timestamp_ns
        elif sample_time <= self._last_command_time:
            self.command_reason = 'duplicate or reordered input'
            return False
        self._last_command_time = sample_time
        values = (vx, vy, omega_deg_s)
        try:
            frame = encode_chassis(vx, vy, math.radians(omega_deg_s), 1)
        except (ValueError, OverflowError):
            self.clear_command()
            self._input_time = None
            self.command_reason = 'non-finite or unrepresentable input'
            return False
        if not any(values):
            self.clear_command()
            self._input_time = sample_time
            self.command_reason = 'zero input'
            return True
        if not ready or sample_time <= self._command_gate:
            self.clear_command()
            self.command_reason = 'input predates recovery or motion unavailable'
            return False
        if ((odom_session_id or motion_epoch) and
                (odom_session_id != self.odom_session_id or motion_epoch != self.motion_epoch)):
            self.clear_command()
            self._input_time = None
            self.command_reason = 'obsolete automatic task binding'
            return False
        self._command, self._command_received = frame, sample_time
        self._input_time = sample_time
        self.command_reason = ''
        return True

    def command_frame(self, now):
        if not self.update(now):
            self.command_reason = self.health_reason(now)
        elif not self.command_fresh(now):
            self.command_reason = 'input timed out' if self._input_time is not None else 'no input'
            self.clear_command()
        if self._command is None:
            return encode_chassis(0., 0., 0., 0)
        return self._command

    def reset_fault(self, now):
        ok = self.update(now)
        return ok, self.health_reason(now) or 'automatic recovery enabled; no reset needed'

    def local_odom(self):
        if self.odom is None or self._origin is None:
            return None
        x0, y0, yaw0 = self._origin
        dx, dy = self.odom.x-x0, self.odom.y-y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        return replace(self.odom, x=c*dx+s*dy, y=-s*dx+c*dy, yaw=self.odom.yaw-yaw0)

    def _accept(self, msg_id, item, now):
        self._malformed.discard(msg_id)
        if msg_id == STATUS:
            old_source = self.status.source if self.status is not None else None
            if item.source == 2 and old_source not in (None, 2):
                self._new_origin(now)
            self.status, self.status_received = item, now
        else:
            self.odom, self.odom_received = item, now
            if (self._origin is None and now >= self._origin_after
                    and self.status_fresh(now) and self.status.source == 2
                    and self.odom_flags_ok(item.flags)):
                self._origin = (item.x, item.y, item.yaw)
            self.odom_sequence += 1
        self.update(now)
        return msg_id == ODOM

    def on_frame(self, msg_id, payload, now):
        if msg_id not in (STATUS, ODOM):
            return False
        self.update(now)  # Detect a gap before the new frame hides it.
        try:
            item = Status(*unpack_status(payload)) if msg_id == STATUS else Odom(*unpack_odom(payload))
        except ValueError:
            self._malformed.add(msg_id)
            self.update(now)
            return False
        previous = self.status if msg_id == STATUS else self.odom
        if previous is not None:
            delta = (item.t_ms-previous.t_ms) & 0xFFFFFFFF
            if delta == 0 and (msg_id == ODOM or item == previous):
                return False
            if delta >= 0x80000000:
                self._restart_candidates[msg_id] = (item, now)
                self.update(now)
                # Require both streams to corroborate a rollback. One reordered
                # packet must never silently replace the coordinate frame.
                if len(self._restart_candidates) < 2:
                    return False
                st, sr = self._restart_candidates[STATUS]
                od, ot = self._restart_candidates[ODOM]
                if abs(sr-ot) > max(self.status_timeout, self.odom_timeout) or abs(st.t_ms-od.t_ms) > 500:
                    return False
                self._new_origin(now)
                self.status = self.odom = None
                self.status_received = self.odom_received = None
                self._restart_candidates.clear()
                self._accept(STATUS, st, now)
                return self._accept(ODOM, od, now)
            received = self.status_received if msg_id == STATUS else self.odom_received
            # Reject delayed pre-reboot frames that leap into the old clock.
            if received is not None and delta > max(2000., (now-received)*1000.+500.):
                return False
        self._restart_candidates.pop(msg_id, None)
        return self._accept(msg_id, item, now)
