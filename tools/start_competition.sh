#!/usr/bin/env bash
# 启动整套比赛；默认前台运行，Ctrl+C 统一退出。不会自动开始运动。
set -eo pipefail
competition_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
competition_action="${1:-run}"
case "$competition_action" in
  -h|--help|help)
    cat <<'HELP'
用法：bash tools/start_competition.sh [start|run|stop|status|logs] [name_config:=/绝对路径/file.yaml ...]
  run     前台启动（默认）；Ctrl+C 统一停止，运行期间保持终端和 SSH 连接
  start   显式后台启动；之后可断开 SSH、关闭笔记本，X5 保持供电
  stop    请求退出整套进程，不强杀或自动续跑
  status  查看进程是否在运行，不代表硬件已就绪
  logs    查看最近 80 行后台日志
首次先在 ros2_ws 构建：colcon build --symlink-install --packages-up-to agv_bringup
雷达开关：ros2_ws/src/agv_mission/config/mission.yaml 中 use_lidar（默认 false）
HELP
    exit 0 ;;
  start|run|stop|status|logs)
    if [[ $# -gt 0 ]]; then shift; fi ;;
  *) echo "未知操作：$competition_action；使用 --help 查看说明" >&2; exit 2 ;;
esac
if [[ ! -f "$competition_repo_root/ros2_ws/install/setup.bash" ]]; then
  echo '请先构建：在 ros2_ws 中运行 colcon build --symlink-install --packages-up-to agv_bringup' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$competition_repo_root/ros2_ws/install/setup.bash"
export AGV_COMPETITION_STATE_DIR="${AGV_COMPETITION_STATE_DIR:-$competition_repo_root/ros2_ws/log/competition}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$AGV_COMPETITION_STATE_DIR/ros}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
# Bind defaults to source YAML so tuning never depends on install copies.
competition_configs=()
if [[ "$competition_action" == start || "$competition_action" == run ]]; then
  competition_source="$competition_repo_root/ros2_ws/src"
  competition_configs=(
    "mission_config:=$competition_source/agv_mission/config/mission.yaml"
    "screen_config:=$competition_source/agv_screen/config/screen.yaml"
    "planner_config:=$competition_source/agv_planning/config/planner.yaml"
    "cboard_config:=$competition_source/agv_cboard/config/cboard.yaml"
    "qr_config:=$competition_source/agv_perception/config/qr.yaml"
    "raw_config:=$competition_source/agv_perception/config/raw.yaml"
    "coarse_config:=$competition_source/agv_perception/config/coarse.yaml"
    "storage_config:=$competition_source/agv_perception/config/storage.yaml"
  )
fi
exec /usr/bin/python3 -m agv_bringup.runtime "$competition_action" "${competition_configs[@]}" "$@"
