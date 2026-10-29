#!/usr/bin/env bash
# 启动现有 mission 与屏幕通信；硬件执行端仍通过各自入口启动。
set -e
task_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ ! -f "$task_repo_root/ros2_ws/install/setup.bash" ]; then
  echo '尚未构建：请在 ros2_ws 中运行 colcon build --symlink-install --packages-up-to agv_screen' >&2
  exit 1
fi
source "$task_repo_root/ros2_ws/install/setup.bash"
exec ros2 launch agv_screen screen_mission.launch.py "$@"
