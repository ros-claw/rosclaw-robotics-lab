#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
uname -m
nvidia-smi --query-gpu=name,driver_version --format=csv
cat "$ISAACSIM_PATH/VERSION"
"$ROSCLAW_SOURCE/.venv/bin/python" -m rosclaw.entrypoint --version
test -x "$ISAACSIM_PATH/isaac-sim.sh"
test -f "$ISAAC_ROS_WS/install/local_setup.bash"
"$CHALLENGE_DIR/scripts/ros-container.sh" bash -c \
  'set -e; ros2 pkg prefix carter_navigation; ros2 pkg prefix isaacsim_bringup; ros2 pkg prefix nav2_bringup; ros2 pkg prefix rosbridge_server; ros2 pkg prefix pointcloud_to_laserscan'
curl --fail --head --silent --show-error --connect-timeout 15 --max-time 30 \
  "$ISAACSIM_ASSET_ROOT/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
echo 'Dependency checks completed. Physics, asset references and navigation still require live validation.'
