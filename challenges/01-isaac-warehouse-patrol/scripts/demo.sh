#!/usr/bin/env bash
# Start the validated simulation environment. No mission or automatic goals.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ -f "$CHALLENGE_DIR/.runtime/sim-process.json" ]]; then
  echo 'Existing project session recorded; run scripts/stop.sh before a new reset.' >&2
  exit 1
fi
run_dir="$CHALLENGE_DIR/reports/runs/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
trap '"$CHALLENGE_DIR/scripts/stop.sh"' ERR
ROSCLAW_LAB_REPORT_DIR="$run_dir" "$CHALLENGE_DIR/scripts/start-sim.sh" "${1:-streaming}" > "$run_dir/scene.log" 2>&1 &
sim_pid=$!
python3 - "$run_dir" "$sim_pid" <<'PY'
import json, sys, time, os
from pathlib import Path
root, pid = Path(sys.argv[1]), int(sys.argv[2])
deadline = time.monotonic() + 1200
while time.monotonic() < deadline:
    os.kill(pid, 0)
    error = root / 'observer-error.json'
    if error.exists():
        raise SystemExit(error.read_text())
    latest = root / 'physics-latest.json'
    if latest.exists():
        data = json.loads(latest.read_text())
        if data['timeline_playing'] and 0 <= time.time() - data['wall_time'] < 2:
            break
    time.sleep(1)
else:
    raise SystemExit('Independent Physics did not become ready in 1200s')
PY
"$CHALLENGE_DIR/scripts/start-nav2.sh" calibration > "$run_dir/nav2.log" 2>&1 &
"$CHALLENGE_DIR/scripts/ros-container.sh" python3 /lab/ros2/wait_navigation.py
printf 'Simulation environment ready. No automatic goals are running.\nEvidence: %s\n' "$run_dir"
