#!/usr/bin/env bash
# Full odometry migration regression. Uses fresh PTYs; never real serial ports.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
export ROS_LOG_DIR=/tmp/agv_cboard_migration_checks
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=93
export AGV_CBOARD_ROS_TEST=1
PYTHONPATH="$PROJECT_DIR/ros2_ws/src/agv_cboard${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s ros2_ws/src/agv_cboard/test -p test_core.py -v
PYTHONPATH="$PROJECT_DIR/ros2_ws/src/agv_cboard${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s ros2_ws/src/agv_cboard/test -p test_relative_move.py -v
PYTHONPATH="$PROJECT_DIR/tools/route_debug${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s tools/route_debug/tests -p test_session.py -v
node tools/route_debug/tests/test_web.js
python3 -m unittest discover -s ros2_ws/src/agv_cboard/test -p test_ros_pty.py -v
export ROS_DOMAIN_ID=92
python3 -m unittest discover -s ros2_ws/src/agv_planning/test -p test_xy_execution.py -v
python3 -m unittest discover -s ros2_ws/src/agv_mission/test -p test_chassis_contract.py -v
exec bash "$PROJECT_DIR/tools/route_debug/tests/run_ros_pty.sh"
