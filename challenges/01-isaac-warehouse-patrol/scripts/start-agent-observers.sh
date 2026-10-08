#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
ROSCLAW_CONTAINER_NAME=rosclaw-warehouse-rosbridge "$CHALLENGE_DIR/scripts/ros-container.sh" ros2 launch /lab/config/rosbridge.launch.xml > "$CHALLENGE_DIR/reports/rosbridge.log" 2>&1 &
ROSCLAW_CONTAINER_NAME=rosclaw-warehouse-probe "$CHALLENGE_DIR/scripts/ros-container.sh" python3 /lab/ros2/native_probe.py --ros-args -p use_sim_time:=true > "$CHALLENGE_DIR/reports/native-probe.log" 2>&1 &
wait
