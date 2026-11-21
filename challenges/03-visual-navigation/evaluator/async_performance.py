"""Audit frame latency, single in-flight vision and simultaneous vehicle motion."""

import argparse
import json
import math
from pathlib import Path
from statistics import median, quantiles
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.controller_run import records


def evaluate(run):
    requests = [row for row in records(run / "motion-audit.jsonl") if row["event"] == "audit_requested"]
    physics = records(run / "physx-trajectory.jsonl")
    captures, starts, ends, pictured = [], [], [], []
    during_motion = 0
    for row in requests:
        directory = Path(row["model_dir"])
        capture = json.loads((directory.parent / "observation_summary.json").read_text())
        response = json.loads((directory / "response.json").read_text())
        observation = json.loads((directory.parent / "observations.jsonl").read_text())
        start, end = (int(response[key]) for key in ("request_host_mono_ns", "received_host_mono_ns"))
        captures.append(capture["wall_seconds"])
        starts.append(start)
        ends.append(end)
        pictured.append((start - int(observation["rgb_receive_mono_ns"])) / 1e9)
        moving = sum(math.hypot(*item["linear_angular_velocity"][:2]) >= 0.05
                     for item in physics if start <= int(item["host_mono_ns"]) <= end)
        during_motion += moving >= 5
    intervals = [(b - a)/1e9 for a,b in zip(starts, starts[1:])]
    nonoverlap = bool(starts) and all(a <= b for a, b in zip(ends, starts[1:]))
    return {"run": run.name, "requests": len(starts), "nonoverlapping_requests": nonoverlap,
            "requests_with_confirmed_motion": during_motion,
            "capture_median_s": median(captures) if captures else None,
            "capture_p95_s": quantiles(captures, n=20, method="inclusive")[18] if len(captures) > 1 else None,
            "frame_receive_to_api_start_median_s": median(pictured) if pictured else None,
            "audit_interval_median_s": median(intervals) if intervals else None,
            "api_latency_median_s": median((b-a)/1e9 for a,b in zip(starts,ends)) if starts else None}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    result = [evaluate(run.resolve()) for run in parser.parse_args().runs]
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if all(r["nonoverlapping_requests"] and r["requests_with_confirmed_motion"] >= 3
                             for r in result) else 1)
