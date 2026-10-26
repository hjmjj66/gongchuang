"""Pure protocol helpers for the RDK X5 ↔ RoboMaster C-board link."""

import math
import struct
import time


SOF = b'\xAA\x55'
CHASSIS_CMD = 0x01
ODOM = 0x81
STATUS = 0x82
ACK = 0x83

CHASSIS_STRUCT = struct.Struct('<fffB3x')
ODOM_STRUCT = struct.Struct('<IffffffB3x')
STATUS_STRUCT = struct.Struct('<IBBBBfffB3x')


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC with the polynomial and initial value specified by the protocol."""
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def encode_frame(msg_id: int, payload: bytes) -> bytes:
    if not 0 <= msg_id <= 0xFF or len(payload) > 0xFF:
        raise ValueError('message ID and payload length must fit in one byte')
    body = bytes((msg_id, len(payload))) + payload
    return SOF + body + struct.pack('<H', crc16(body))


def encode_chassis(vx: float, vy: float, wz: float, mode: int) -> bytes:
    values = (float(vx), float(vy), float(wz))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('chassis values must be finite')
    if mode not in (0, 1):
        raise ValueError('mode must be 0 or 1')
    return encode_frame(CHASSIS_CMD, CHASSIS_STRUCT.pack(*values, mode))


class FrameParser:
    """Incremental parser which resynchronizes after noise and bad CRCs."""

    def __init__(self, max_buffer: int = 4096, partial_timeout: float = 0.1):
        if max_buffer < 261 or not math.isfinite(partial_timeout) or partial_timeout <= 0:
            raise ValueError('parser needs capacity for a 261-byte frame and a positive timeout')
        self._buffer = bytearray()
        self._max_buffer = max_buffer
        self._partial_timeout = partial_timeout
        self._partial_since = None

    def reset(self):
        self._buffer.clear()
        self._partial_since = None

    def feed(self, data: bytes, now=None):
        now = time.monotonic() if now is None else now
        if (self._partial_since is not None
                and now - self._partial_since > self._partial_timeout):
            # A corrupted LEN must not hold later complete frames indefinitely.
            del self._buffer[:1]
            self._partial_since = None
        frames = []
        # Drain complete frames before consuming more bytes. Truncating an
        # oversized read first would silently discard valid leading frames.
        # A drained buffer retains at most 260 bytes (one incomplete frame).
        chunk_size = self._max_buffer - 260
        for offset in range(0, len(data), chunk_size):
            self._buffer.extend(data[offset:offset + chunk_size])
            frames.extend(self._drain(now))
        if not data:
            frames.extend(self._drain(now))
        return frames

    def _drain(self, now):
        frames = []
        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                # Retain a possible first byte of the two-byte SOF.
                if self._buffer[-1:] == SOF[:1]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                self._partial_since = None
                break
            if start:
                del self._buffer[:start]
                self._partial_since = None
            if len(self._buffer) < 4:
                if self._partial_since is None:
                    self._partial_since = now
                break
            length = self._buffer[3]
            frame_length = 6 + length
            if len(self._buffer) < frame_length:
                if self._partial_since is None:
                    self._partial_since = now
                break
            self._partial_since = None
            body = bytes(self._buffer[2:4 + length])
            expected = struct.unpack_from('<H', self._buffer, 4 + length)[0]
            if crc16(body) != expected:
                # Shift by one byte, rather than discarding a possible nested SOF.
                del self._buffer[0]
                continue
            frames.append((body[0], bytes(body[2:])))
            del self._buffer[:frame_length]
        return frames


def unpack_odom(payload: bytes):
    if len(payload) < ODOM_STRUCT.size:
        raise ValueError(f'ODOM payload too short: {len(payload)}')
    values = ODOM_STRUCT.unpack_from(payload)
    if not all(math.isfinite(value) for value in values[1:7]):
        raise ValueError('ODOM contains a non-finite float')
    return values


def unpack_status(payload: bytes):
    if len(payload) < STATUS_STRUCT.size:
        raise ValueError(f'STATUS payload too short: {len(payload)}')
    values = STATUS_STRUCT.unpack_from(payload)
    if not all(math.isfinite(value) for value in values[5:8]):
        raise ValueError('STATUS contains a non-finite float')
    return values
