"""Linux process lifecycle for a detached launch; no hardware imports or auto restart."""

import argparse
from collections import deque
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


class Runtime:
    def __init__(self, directory, command):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.command = command
        self.child = None
        self.lock_path = self.directory / 'run.lock'
        self.pid_path = self.directory / 'launch.pid'
        self.log_path = self.directory / 'competition.log'

    def pid(self):
        with self.lock_path.open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                pass
            else:
                return None  # An old PID file alone never establishes ownership.
        try:
            pid = int(self.pid_path.read_text().strip())
            if pid <= 1:
                raise ValueError('invalid PID')
            inode = self.lock_path.stat()
            for fd in Path(f'/proc/{pid}/fd').iterdir():
                try:
                    target = fd.stat()
                except FileNotFoundError:
                    continue
                if (target.st_dev, target.st_ino) == (inode.st_dev, inode.st_ino):
                    return pid
        except (OSError, ValueError):
            pass
        raise RuntimeError('运行锁已占用但无法确认进程身份；未启动第二份，也未发送停止信号')

    def run(self):
        lock = self.lock_path.open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise RuntimeError('比赛启动器已在运行')
        # exec preserves this descriptor, so the lock belongs to ros2 launch for
        # its entire lifetime. It is released by the kernel even after a crash.
        os.set_inheritable(lock.fileno(), True)
        self.pid_path.write_text(str(os.getpid()) + '\n')
        try:
            os.execvp(self.command[0], self.command)
        finally:
            lock.close()

    def start(self, runner):
        current = self.pid()
        if current is not None:
            print(f'比赛进程已在运行，PID={current}；未重复启动')
            return
        with self.log_path.open('a') as log:
            log.write(f'\n--- 启动 {time.strftime("%Y-%m-%d %H:%M:%S %z")} ---\n')
            log.flush()
            child = subprocess.Popen(runner, stdin=subprocess.DEVNULL, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            self.child = child
        # This acknowledges launch startup, never hardware readiness or movement.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError(f'启动进程已退出（{child.returncode}）；查看 {self.log_path}')
            try:
                if self.pid() == child.pid:
                    print(f'后台进程已创建，PID={child.pid}；可断开 SSH。')
                    print(f'就绪和故障请查看日志：{self.log_path}')
                    return
            except RuntimeError:
                pass  # Child has acquired its lock but has not written its PID yet.
            time.sleep(0.05)
        raise RuntimeError(f'未在 5 秒内确认启动；请查看 {self.log_path}，不要另开控制进程')

    def stop(self, timeout=20.0):
        current = self.pid()
        if current is None:
            print('比赛启动器未运行')
            return
        try:
            os.kill(current, signal.SIGINT)  # launch propagates shutdown to its children.
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pid() is None:
                if self.child is not None and self.child.pid == current:
                    self.child.wait(timeout=1.0)
                print('比赛启动器已退出；车辆和机构状态仍需现场确认')
                return
            time.sleep(0.1)
        raise RuntimeError('停止尚未确认；未强杀进程，请检查日志和执行端')


def main():
    parser = argparse.ArgumentParser(description='整套比赛后台启动和日志管理')
    parser.add_argument('action', choices=('start', 'run', 'stop', 'status', 'logs'))
    parser.add_argument('launch_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.launch_args and args.action not in ('start', 'run'):
        parser.error('只有 start/run 接受 ROS launch 参数')
    directory = os.environ.get('AGV_COMPETITION_STATE_DIR',
                               str(Path.home() / '.local/state/agv-competition'))
    runtime = Runtime(directory, ['ros2', 'launch', 'agv_bringup', 'competition.launch.py', *args.launch_args])
    try:
        if args.action == 'run':
            runtime.run()
        elif args.action == 'logs':
            if not runtime.log_path.exists():
                print(f'尚无后台日志：{runtime.log_path}')
            else:
                with runtime.log_path.open(errors='replace') as stream:
                    print(''.join(deque(stream, maxlen=80)), end='')
        else:
            # Serialize manager commands; the runtime lock is held by launch.
            with (runtime.directory / 'manager.lock').open('a') as manager:
                fcntl.flock(manager, fcntl.LOCK_EX)
                if args.action == 'start':
                    runtime.start([sys.executable, '-m', 'agv_bringup.runtime', 'run', *args.launch_args])
                elif args.action == 'stop':
                    runtime.stop()
                else:
                    pid = runtime.pid()
                    print(f'运行中，PID={pid}；不代表执行端 READY' if pid else '未运行')
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    main()
