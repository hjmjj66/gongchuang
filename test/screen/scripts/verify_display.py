#!/usr/bin/env python3
"""Standalone TJC text verification; no ROS or chassis commands."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
import time

import serial


END = b'\xff\xff\xff'
TARGET = '156+123+516+231'
ERRORS = {
    0x00: 'invalid instruction', 0x02: 'invalid component ID',
    0x03: 'invalid page ID', 0x04: 'invalid picture ID',
    0x05: 'invalid font ID', 0x1A: 'invalid variable name',
    0x1B: 'invalid variable operation', 0x1C: 'assignment failed',
}


class Screen:
    def __init__(self, port, baud, record):
        # Leave modem lines low before opening the USB adapter.
        self.ser = serial.Serial(port=None, baudrate=baud, timeout=0.05,
                                 write_timeout=1, exclusive=True)
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.port = port
        self.ser.open()
        self.record = record
        self.pending = bytearray()

    def log(self, kind, **values):
        row = {'time_utc': datetime.now(timezone.utc).isoformat(),
               'monotonic_s': time.monotonic(), 'kind': kind, **values}
        self.record.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.record.flush()

    def receive(self, seconds):
        frames = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = self.ser.read(min(self.ser.in_waiting or 1, 4096))
            if not chunk:
                continue
            self.log('rx_bytes', hex=chunk.hex(' '))
            self.pending.extend(chunk)
            while END in self.pending:
                pos = self.pending.index(END)
                frame = bytes(self.pending[:pos])
                del self.pending[:pos + len(END)]
                if not frame:
                    continue
                frames.append(frame)
                description = repr(frame)
                if len(frame) == 1 and frame[0] in ERRORS:
                    description = ERRORS[frame[0]]
                print('  RX', frame.hex(' '), description, flush=True)
        return frames

    def command(self, command, wait=0.6):
        print('TX', command, flush=True)
        payload = command.encode('ascii') + END
        self.log('tx', command=command, hex=payload.hex(' '))
        if self.ser.write(payload) != len(payload):
            raise RuntimeError('Incomplete serial write')
        self.ser.flush()
        return self.receive(wait)

    def text(self, control):
        frames = self.command(f'get {control}.txt')
        values = [f[1:].decode('ascii') for f in frames if f[:1] == b'\x70']
        return values[-1] if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--write', action='store_true', help='write target and verify readback')
    parser.add_argument('--allow-unconfirmed-write', action='store_true',
                        help='send target even without identification/control readback')
    parser.add_argument('--layout', choices=('single', 'split'), default='split')
    parser.add_argument('--controls', nargs=2, default=['t0', 't1'])
    args = parser.parse_args()
    if args.allow_unconfirmed_write and not args.write:
        parser.error('--allow-unconfirmed-write requires --write')
    if args.baud <= 0 or not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.]*', c)
                                for c in args.controls):
        parser.error('Invalid baud or control names')
    root = Path(__file__).resolve().parents[1]
    records = root / 'records'
    records.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    path = records / f'display_{stamp}.jsonl'
    screen = None
    with path.open('x', encoding='utf-8') as record:
        try:
            screen = Screen(args.port, args.baud, record)
            screen.log('start', port=args.port, baud=args.baud, write=args.write,
                       layout=args.layout, controls=args.controls, target=TARGET)
            screen.receive(0.3)
            # Empty terminated command first aligns a possibly partial input.
            screen.command('', wait=0.15)
            replies = screen.command('connect', wait=1.0)
            identified = any(b'comok' in frame for frame in replies)
            if identified:
                screen.command('sendme')
            # Existing controls can confirm TJC communication even if connect is unsupported.
            before = {c: screen.text(c) for c in args.controls}
            print('Current text:', json.dumps(before, ensure_ascii=False), flush=True)
            if (not identified and all(v is None for v in before.values())
                    and not args.allow_unconfirmed_write):
                screen.log('result', status='NO_SCREEN_RESPONSE')
                print('No TJC identification or text response. No text was written.')
                return 2
            if not args.write:
                screen.log('result', status='PROBE_ONLY', identified=identified, text=before)
                print('Probe finished. No display text was changed.')
                return 0
            pairs = ([(args.controls[0], TARGET)] if args.layout == 'single'
                     else list(zip(args.controls, ('156+123', '516+231'))))
            if any(before[c] is None for c, _ in pairs) and not args.allow_unconfirmed_write:
                screen.log('result', status='MISSING_TEXT_CONTROL', before=before)
                print('Required text controls are missing; create/confirm the screen page first.')
                return 3
            if args.allow_unconfirmed_write:
                screen.log('unconfirmed_write', before=before)
                print('Sending to requested controls without confirmed readback; '
                      'this alone cannot pass acceptance.', flush=True)
            for control, value in pairs:
                screen.command(f'{control}.txt="{value}"')
            actual = {control: screen.text(control) for control, _ in pairs}
            matched = all(actual[c] == value for c, value in pairs)
            screen.log('result', status='READBACK_OK' if matched else 'READBACK_FAILED',
                       expected=dict(pairs), actual=actual, visual_acceptance='PENDING')
            print('Readback:', json.dumps(actual, ensure_ascii=False))
            print('READBACK_OK; visual confirmation still required.' if matched
                  else 'READBACK_FAILED; acceptance has not passed.')
            return 0 if matched else 4
        except (serial.SerialException, OSError, ValueError, RuntimeError) as exc:
            if screen:
                screen.log('error', message=str(exc))
            print(f'ERROR: {exc}', file=sys.stderr)
            return 1
        finally:
            if screen:
                screen.ser.close()
            print('Record:', path, flush=True)


if __name__ == '__main__':
    sys.exit(main())
