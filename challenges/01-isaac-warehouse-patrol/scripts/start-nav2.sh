#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
"$CHALLENGE_DIR/scripts/ros-container.sh" timeout 60 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --field clock
export ROSCLAW_CONTAINER_NAME=rosclaw-warehouse-nav2
nav_args=()
if [[ "${1:-official}" == "calibration" ]]; then
  nav_args=(params_file:=/lab/config/calibration_navigation_params.yaml)
elif [[ "${1:-official}" == "patrol" ]]; then
  nav_args=(params_file:=/lab/config/patrol_navigation_params.yaml)
elif [[ "${1:-official}" != "official" ]]; then
  echo "Usage: start-nav2.sh [official|calibration|patrol]" >&2; exit 2
fi
exec "$CHALLENGE_DIR/scripts/ros-container.sh" ros2 launch /lab/config/headless_nav.launch.xml "${nav_args[@]}"
