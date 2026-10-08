#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
container_args=()
if [[ -n "${ROSCLAW_CONTAINER_NAME:-}" ]]; then container_args=(--name "$ROSCLAW_CONTAINER_NAME"); fi
exec docker run "${container_args[@]}" --label "rosclaw.lab.root=$CHALLENGE_DIR" --rm --network host --ipc host --user "$(id -u):$(id -g)" \
  -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION -e ROS_LOCALHOST_ONLY \
  -e HOME=/tmp/rosclaw-lab \
  -v "$ISAAC_ROS_WS:/work" -v "$CHALLENGE_DIR:/lab" -v "$ROSCLAW_SOURCE:/rosclaw_source:ro" -w /work \
  "$ROS_CONTAINER_IMAGE" bash -c \
  'source /opt/ros/jazzy/setup.bash; source /work/install/local_setup.bash; exec "$@"' bash "$@"
