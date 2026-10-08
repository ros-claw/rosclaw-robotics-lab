"""Independent accuracy check for the official three-goal environment baseline.

This does not verify an Agent patrol, dwell, Home return or collision acceptance.
"""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path


def evaluate(trajectory, goal_log, nav_log, tolerance=0.4):
    results = re.findall(r"\[([0-9.]+)\].*Result:.*error_code=(\d+)", goal_log)
    successes = re.findall(r"\[([0-9.]+)\].*\[bt_navigator\]: Goal succeeded", nav_log)
    expected = [(-4, -1), (-4, 1), (-6, 1)]
    checks = []
    for goal, (stamp, code) in zip(expected, results):
        stamp = float(stamp)
        if not trajectory:
            checks.append({"goal_xy": goal, "status": "UNKNOWN"})
            continue
        sample = min(trajectory, key=lambda s: abs(s["wall_time"] - stamp))
        offset = abs(sample["wall_time"] - stamp)
        position = sample["physics_transforms_xyzw"][0][:2]
        error = math.dist(goal, position)
        observed = offset <= 0.5 and sample["timeline_playing"] is True
        nav_success = int(code) == 0 and any(
            abs(float(s) - stamp) < 0.5 for s in successes
        )
        checks.append(
            {
                "goal_xy": goal,
                "physical_xy": position,
                "error_m": error,
                "sample_offset_s": offset,
                "nav2_success": nav_success,
                "status": "UNKNOWN"
                if not observed
                else "PASS"
                if nav_success and math.isfinite(error) and error <= tolerance
                else "FAIL",
            }
        )
    status = (
        "UNKNOWN"
        if len(results) != 3
        or len(checks) != 3
        or any(c["status"] == "UNKNOWN" for c in checks)
        else "PASS"
        if all(c["status"] == "PASS" for c in checks)
        else "FAIL"
    )
    return {
        "scope": "official navigation baseline only",
        "accuracy_status": status,
        "tolerance_m": tolerance,
        "goals": checks,
        "collision_status": "UNKNOWN; raw contacts require floor validation",
        "agent_patrol_status": "NOT_RUN",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--goals", type=Path, required=True)
    parser.add_argument("--nav", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.trajectory.read_text().splitlines()]
    report = evaluate(rows, args.goals.read_text(), args.nav.read_text())
    report["sources"] = [
        {"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
        for p in [args.trajectory, args.goals, args.nav]
    ]
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["accuracy_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
