#!/usr/bin/env bash
# Project dependencies only. No NVIDIA driver/CUDA or host ROS replacement.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if ! docker image inspect "$ROS_CONTAINER_IMAGE" >/dev/null 2>&1; then
  docker build --platform linux/arm64 -t "$ROS_CONTAINER_IMAGE" -f "$CHALLENGE_DIR/docker/Dockerfile" "$CHALLENGE_DIR/docker"
fi
workspace_root=$(dirname -- "$ISAAC_ROS_WS")
if [[ ! -d "$workspace_root/.git" ]]; then
  git clone --recurse-submodules --branch IsaacSim-6.1.0 https://github.com/isaac-sim/IsaacSim-ros_workspaces.git "$workspace_root"
fi
actual_commit=$(git -C "$workspace_root" rev-parse HEAD)
test "$actual_commit" = a9e8471ee901bc2332c1e4aca94ac580713ca3ab || { echo 'Workspace revision differs from pinned 6.1.0.' >&2; exit 1; }
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp/rosclaw-lab \
  -v "$ISAAC_ROS_WS:/work" -w /work "$ROS_CONTAINER_IMAGE" bash -c \
  'source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-up-to carter_navigation isaacsim_bringup isaac_ros_navigation_goal'
if [[ ! -d "$ROSCLAW_SOURCE/.git" ]]; then
  git clone https://github.com/ros-claw/rosclaw.git "$ROSCLAW_SOURCE"
  git -C "$ROSCLAW_SOURCE" checkout 21838614bb14c39599b8acef731b2b64dad3b92a
fi
test "$(git -C "$ROSCLAW_SOURCE" rev-parse HEAD)" = 21838614bb14c39599b8acef731b2b64dad3b92a || { echo 'ROSClaw source revision differs; choose a separate pinned checkout.' >&2; exit 1; }
if [[ ! -x "$ROSCLAW_SOURCE/.venv/bin/python" ]]; then
  uv venv --python 3.12 "$ROSCLAW_SOURCE/.venv"
  uv pip install --python "$ROSCLAW_SOURCE/.venv/bin/python" -e "$ROSCLAW_SOURCE"
fi
if [[ ! -f "$ROSCLAW_SOURCE/packages/rosclaw-agent/dist/src/main.js" ]]; then
  (cd "$ROSCLAW_SOURCE/packages/rosclaw-agent"; npm ci; npm run build)
fi
printf 'Project dependencies prepared. Configure a working ROSClaw model before Native acceptance.\n'
