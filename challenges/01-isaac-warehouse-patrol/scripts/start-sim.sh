#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
scene_uri=$(python3 "$CHALLENGE_DIR/isaac/asset_resolver.py" --scene)
export ISAACSIM_ASSET_ROOT="${scene_uri%/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd}"
startup_script="$ISAAC_ROS_WS/src/isaacsim_bringup/scripts/open_isaacsim_stage.py"
test -f "$startup_script"
# Use packaged ROS libraries; keep simulation and Nav2 in the same isolated DDS domain.
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export ROSCLAW_LAB_CHALLENGE_DIR="$CHALLENGE_DIR"
export ROSCLAW_LAB_REPORT_DIR="${ROSCLAW_LAB_REPORT_DIR:-$CHALLENGE_DIR/reports}"
case "${1:-headless}" in
  headless) launcher="$ISAACSIM_PATH/isaac-sim.sh"; args=(--no-window) ;;
  streaming) launcher="$ISAACSIM_PATH/isaac-sim.streaming.sh"; args=() ;;
  gui) launcher="$ISAACSIM_PATH/isaac-sim.sh"; args=() ;;
  *) echo 'Usage: start-sim.sh [headless|streaming|gui]' >&2; exit 2 ;;
esac
mkdir -p "$CHALLENGE_DIR/.runtime"
python3 - "$CHALLENGE_DIR/.runtime/sim-process.json" "$$" <<'PYMETA'
import json, sys
from pathlib import Path
pid = int(sys.argv[2])
stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
Path(sys.argv[1]).write_text(json.dumps({"pid": pid, "birth_ticks": stat[19]}))
PYMETA
exec "$launcher" "${args[@]}" \
  --/exts/isaacsim.core.simulation_manager/default_engine=physx \
  --/exts/isaacsim.physics.newton/auto_switch_on_startup=false \
  --/renderer/raytracingMotion/enabled=true \
  --/app/renderer/skipWhileInvisible=false \
  --/app/renderer/skipWhileMinimized=false \
  --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
  --exec "$startup_script --path $scene_uri --start-on-play --python-script $CHALLENGE_DIR/isaac/baseline_observer.py"
