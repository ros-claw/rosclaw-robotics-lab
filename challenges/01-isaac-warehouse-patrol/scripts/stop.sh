#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
python3 - "$CHALLENGE_DIR" <<'PY_STOP'
import json, os, signal, subprocess, sys, time
from pathlib import Path
root = Path(sys.argv[1])
containers = subprocess.check_output(["docker", "ps", "--filter", "label=rosclaw.lab.root=" + str(root), "-q"], text=True).split()
for container in containers:
    result = subprocess.run(["docker", "stop", "--timeout", "10", container], capture_output=True, text=True)
    if result.returncode and "No such container" not in result.stderr:
        raise RuntimeError(result.stderr.strip())
metadata = root / ".runtime/sim-process.json"
if metadata.exists():
    saved = json.loads(metadata.read_text())
    proc = Path("/proc") / str(saved["pid"])
    if proc.exists():
        stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        cmd = (proc / "cmdline").read_bytes()
        if stat[19] != saved["birth_ticks"] or b"/isaacsim/kit/kit" not in cmd:
            raise SystemExit("Process identity differs; refusing to stop it.")
        os.kill(saved["pid"], signal.SIGINT)
        deadline = time.monotonic() + 20
        while proc.exists() and time.monotonic() < deadline:
            try:
                if (proc / "stat").read_text().rsplit(")", 1)[1].split()[0] == "Z":
                    break
            except FileNotFoundError:
                break
            time.sleep(0.2)
        else:
            if proc.exists():
                raise SystemExit("Simulator did not stop before deadline; process metadata retained.")
    metadata.unlink()
print("Project-owned processes stopped.")
PY_STOP
