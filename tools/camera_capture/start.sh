#!/usr/bin/env bash
set -euo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$TOOL_DIR/../.." && pwd)"
export PYTHONPATH="$PROJECT_DIR/ros2_ws/src/agv_perception${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$TOOL_DIR/server.py" "$@"
