#!/usr/bin/env bash
# Test orchestration only: task interpretation and site selection remain in Native Agent.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
run_dir=${1:?Usage: run-native-acceptance.sh RUN_DIRECTORY PHYSICS_DIRECTORY TASK SITE...}
physics_dir=${2:?}
task_text=${3:?}
shift 3
extra_contract=()
if [[ "${ROSCLAW_REQUIRE_OBSTACLE_EVIDENCE:-0}" == 1 ]]; then
  extra_contract+=(--require-obstacle-evidence)
fi
"$ROSCLAW_SOURCE/.venv/bin/python" "$CHALLENGE_DIR/rosclaw/prepare.py" "${extra_contract[@]}" \
  --directory "$run_dir" --physics "$physics_dir" --task "$task_text" --expected-order "$@" --use-configured-model
run_dir=$(cd -- "$run_dir" && pwd)
exec "$ROSCLAW_SOURCE/.venv/bin/python" "$CHALLENGE_DIR/rosclaw/native.py" --directory "$run_dir"
