#!/usr/bin/env bash
set -eo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$TOOL_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]]; then
  echo '请先按 tools/competition_debug/README.md 构建工作空间。' >&2
  exit 1
fi
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export PYTHONPATH="$TOOL_DIR:$PROJECT_DIR/tools/route_debug:$PROJECT_DIR/tools/coarse_debug:$PROJECT_DIR/ros2_ws/src/agv_perception:$PROJECT_DIR/ros2_ws/src/agv_mission${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 -c 'from agv_interfaces.msg import ArmServoCommand, RouteExecutionProgress; from agv_interfaces.srv import ArmCommand; from agv_screen.protocol import ScreenMessageParser'
/usr/bin/python3 -m competition_debug.node --check
exec ros2 launch "$TOOL_DIR/launch/competition_debug.launch.py" "$@"
