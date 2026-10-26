#!/usr/bin/env bash
# Isolated ROS/HTTP/PTY verification. This entry never uses /dev/rm_cboard.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export ROS_LOG_DIR=/tmp/agv_route_debug_test
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=94
export AGV_ROUTE_DEBUG_ROS_TEST=1
export PYTHONPATH="$PROJECT_DIR/tools/route_debug${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m unittest discover -s "$PROJECT_DIR/tools/route_debug/tests" -p test_preflight.py -v
exec /usr/bin/python3 -m unittest discover -s "$PROJECT_DIR/tools/route_debug/tests" -p test_ros_pty.py -v
