#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
test -f "$ISAAC_ROS_WS/src/navigation/carter_navigation/package.xml"
actual_commit=$(git -C "$ISAAC_ROS_WS/.." rev-parse HEAD)
test "$actual_commit" = a9e8471ee901bc2332c1e4aca94ac580713ca3ab || { echo 'Workspace revision differs from pinned 6.1.0.' >&2; exit 1; }
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp/rosclaw-lab \
  -v "$ISAAC_ROS_WS:/work" -w /work "$ROS_CONTAINER_IMAGE" bash -c \
  'source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-up-to carter_navigation isaacsim_bringup isaac_ros_navigation_goal'
