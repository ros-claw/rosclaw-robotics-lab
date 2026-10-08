#!/usr/bin/env bash
set -eo pipefail
CHALLENGE_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set -a
source "$CHALLENGE_DIR/config/runtime.env"
set +a
export ROS_LOCALHOST_ONLY=1
export ISAACSIM_ASSET_ROOT=$(python3 "$CHALLENGE_DIR/isaac/asset_resolver.py")
mkdir -p "$CHALLENGE_DIR/reports"
