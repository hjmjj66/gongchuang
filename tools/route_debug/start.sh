#!/usr/bin/env bash
set -eo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$TOOL_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]]; then
  echo '请先按 tools/route_debug/README.md 构建工作空间。' >&2
  exit 1
fi
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export PYTHONPATH="$TOOL_DIR${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -c 'from agv_interfaces.msg import RouteExecutionProgress' || {
  echo '接口未更新，请先重新构建 agv_interfaces、agv_planning 和 agv_cboard。' >&2
  exit 1
}
/usr/bin/python3 -m route_debug.node --check
exec ros2 launch "$TOOL_DIR/launch/route_debug.launch.py" "$@"
