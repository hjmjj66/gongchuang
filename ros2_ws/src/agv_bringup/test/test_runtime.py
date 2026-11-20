"""Test detached ownership and shutdown using a harmless process, never ROS hardware."""

import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

from agv_bringup.runtime import Runtime


class RuntimeTests(unittest.TestCase):
    def test_detached_start_duplicate_and_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'child.json'
            code = ('import json,os,signal,time; '
                    'signal.signal(signal.SIGINT, lambda *_: exit(0)); '
                    f'open({str(marker)!r}, "w").write(json.dumps([os.getpid(),os.getsid(0)])); '
                    'time.sleep(60)')
            runtime = Runtime(directory, [])
            runner = [sys.executable, str(Path(__file__).resolve()), '--child', directory, code]
            runtime.start(runner)
            try:
                deadline = time.monotonic() + 5
                while not marker.exists():
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(0.02)
                pid, session = json.loads(marker.read_text())
                self.assertEqual(runtime.pid(), pid)
                self.assertEqual(session, pid)  # Detached from the invoking terminal session.
                runtime.start(runner)
                self.assertEqual(runtime.pid(), pid)
                runtime.stop(timeout=5)
                self.assertIsNone(runtime.pid())
            finally:
                if runtime.pid():
                    runtime.stop(timeout=5)

    def test_stale_pid_file_never_targets_an_unrelated_process(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Runtime(directory, [])
            runtime.pid_path.write_text(str(os.getpid()))
            self.assertIsNone(runtime.pid())
            runtime.stop(timeout=0)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--child':
        Runtime(sys.argv[2], [sys.executable, '-c', sys.argv[3]]).run()
    else:
        unittest.main()
