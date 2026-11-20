#!/usr/bin/env bash
set -eo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="$(cd -- "$TOOL_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export PYTHONPATH="$TOOL_DIR:$PROJECT_DIR/tools/route_debug:$PROJECT_DIR/tools/coarse_debug:$PROJECT_DIR/ros2_ws/src/agv_mission:$PROJECT_DIR/ros2_ws/src/agv_perception:$PROJECT_DIR/ros2_ws/src/agv_perception/test${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-211}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/agv_competition_debug_tests}"
if [[ "${1:-}" == '--ros-pty' ]]; then
  export AGV_COMPETITION_DEBUG_ROS_TEST=1
fi
python3 -m unittest discover -s "$TOOL_DIR/tests" -p 'test_*.py' -v
node "$TOOL_DIR/tests/test_web.js"
