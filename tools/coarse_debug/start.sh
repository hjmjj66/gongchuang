#!/usr/bin/env bash
set -eo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$TOOL_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]]; then
  echo '请先按 tools/coarse_debug/README.md 构建工作空间。' >&2
  exit 1
fi
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export PYTHONPATH="$TOOL_DIR:$PROJECT_DIR/tools/route_debug:$PROJECT_DIR/ros2_ws/src/agv_perception${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 -m coarse_debug.node --check
exec ros2 launch "$TOOL_DIR/launch/coarse_debug.launch.py" "$@"
