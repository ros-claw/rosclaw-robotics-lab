#!/usr/bin/env bash
# One natural-language input to the real Native Agent. No automatic goal sender.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
run_dir=$(cd -- "${1:?Usage: start-rosclaw.sh RUN_DIRECTORY}" && pwd)
test -S "$run_dir/run/rosclawd.sock"
cd -- "$run_dir"
export ROSCLAW_HOME="$run_dir/home"
export ROSCLAW_DAEMON_SOCKET="$run_dir/run/rosclawd.sock"
export ROSCLAW_ROS_EXPERT=1
exec "$ROSCLAW_SOURCE/.venv/bin/python" -m rosclaw.entrypoint chat
