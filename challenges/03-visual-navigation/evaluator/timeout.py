"""Verify that a deliberately timed-out live Luna request did not start motion."""

import argparse
import json
import math
from pathlib import Path


def evaluate(model_dir):
    run = model_dir.parents[1]
    reply = json.loads((model_dir / "response.json").read_text())
    started = int(reply["request_host_mono_ns"])
    deadline = int(reply["received_host_mono_ns"])
    samples = [json.loads(line) for line in (run / "physx-trajectory.jsonl").read_text().splitlines()]
    before = [row for row in samples if int(row["host_mono_ns"]) <= started]
    after = [row for row in samples if int(row["host_mono_ns"]) > started]
    if not before or not after:
        raise ValueError("expected physical coverage before and after the request")
    window = [before[-1], *after]
    stamps = [row["sim_time_s"] for row in window]
    stationary = [
        max(math.hypot(*row["linear_angular_velocity"][:2]) for row in window),
        max(abs(row["linear_angular_velocity"][5]) for row in window),
    ]
    coverage = (int(window[-1]["host_mono_ns"]) - deadline) / 1e9
    gaps = max(b - a for a, b in zip(stamps, stamps[1:]))
    controller_files = sorted(item.name for item in run.glob("controller-*.jsonl"))
    failure = reply.get("failure")
    passed = (failure in {"transport_curl_28", "transport_deadline"} and not controller_files
              and coverage >= 2 and stamps[-1] - stamps[0] >= 1 and gaps <= 0.15
              and stationary[0] <= 0.03 and stationary[1] <= 0.08
              and all(item["source"] == "physx" for item in window))
    return {"pass": passed, "observed_failure": failure, "samples": len(window),
            "sim_span_s": stamps[-1] - stamps[0], "post_timeout_wall_s": coverage,
            "largest_sample_gap_sim_s": gaps, "peak_linear_mps": stationary[0],
            "peak_angular_radps": stationary[1], "controller_event_files": controller_files}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_dir", type=Path)
    args = parser.parse_args()
    report = evaluate(args.model_dir)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 1)
