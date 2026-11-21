"""Audit motion through FollowPath replacement and through the committed gate."""

import argparse
import json
import math
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluator.controller_run import evaluate as evaluate_route
from evaluator.controller_run import records
from navigation.candidates import Gate, costmap_footprint_radius


def motion_window(samples, time_ns):
    radius_ns = 350_000_000
    rows = [r for r in samples if abs(int(r["host_mono_ns"]) - time_ns) <= radius_ns]
    stamps = [float(r["sim_time_s"]) for r in rows]
    wall_times = [int(r["host_mono_ns"]) for r in rows]
    speeds = [math.hypot(*r["linear_angular_velocity"][:2]) for r in rows]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    gap = max(gaps, default=None)
    low_speed = min(speeds, default=None)
    passed = (len(rows) >= 5 and wall_times[0] <= time_ns - 175_000_000
              and wall_times[-1] >= time_ns + 175_000_000
              and all(b > a for a, b in zip(wall_times, wall_times[1:]))
              and min(gaps) > 0 and gap <= 0.15 and low_speed >= 0.05
              and all(r["source"] == "physx" for r in rows))
    return {"pass": passed, "samples": len(rows), "minimum_linear_mps": low_speed,
            "largest_sample_gap_sim_s": gap}


def minimum_corridor_clearance(trajectory, gate, radius):
    return min((min(y - gate.lower_y, gate.upper_y - y) - radius
                for x, y, *_ in (row["pose_xyz_xyzw"] for row in trajectory)
                if gate.entry_x - radius <= x <= gate.exit_x + radius), default=None)


def evaluate(run, controller, layout):
    route = evaluate_route(run, controller, layout)
    events = records(controller)
    samples = records(run / "physx-trajectory.jsonl")
    accepted = [e for e in events if e["event"] == "nav2_accepted"]
    validated = [e for e in events if e["event"] == "path_validated"]
    ignored = [e for e in events if e["event"] == "nav2_result_ignored"]
    discarded = [e for e in events if e["event"] == "planning_discarded"]
    held = [e for e in events if e["event"] == "planning_held"]
    terminals = [e for e in events if e["event"] == "nav2_result"]
    transitions = [{"path_seq": e["path_seq"], **motion_window(samples, int(e["host_mono_ns"]))}
                   for e in accepted[1:]]
    gate_motion = None
    clearance = None
    if validated and accepted and route.get("physical_gate_crossings"):
        first_crossing = route["physical_gate_crossings"][0]
        x = first_crossing["xy_m"][0]
        first_ack = int(accepted[0]["host_mono_ns"])
        stop = next(e for e in events if e["event"] == "odometry_stopped")
        end = int(stop["host_mono_ns"]) + 3_000_000_000
        run_samples = [s for s in samples if first_ack <= int(s["host_mono_ns"]) <= end]
        public = json.loads(layout.read_text())
        gate = next(Gate(**g) for g in public["gates"] if g["gate_id"] == validated[0]["gate_id"])
        params = yaml.safe_load(layout.with_name("params.yaml").read_text())
        clearance = minimum_corridor_clearance(run_samples, gate, costmap_footprint_radius(params))
        for a, b in zip(run_samples, run_samples[1:]):
            xa, xb = a["pose_xyz_xyzw"][0], b["pose_xyz_xyzw"][0]
            if xa < x <= xb:
                ta, tb = int(a["host_mono_ns"]), int(b["host_mono_ns"])
                gate_motion = motion_window(samples, round(ta + (tb - ta) * (x - xa) / (xb - xa)))
                break
    applied = (run / "nav2.log").read_text().count("Passing new path to controller")
    sequence_ok = (
        [e["path_seq"] for e in accepted] == [1, 3, 4]
        and len({e["goal_id"] for e in accepted}) == 3
        and all(e["route_version"] == 1 for e in accepted)
        and [e["path_seq"] for e in validated] == [1, 3, 4]
        and len({json.dumps(e["points"]) for e in validated}) == 3
        and len({e["gate_id"] for e in validated}) == 1
        and [(e["path_seq"], e["active_path_seq"]) for e in ignored] == [(1, 3), (3, 4)]
        and [e["path_seq"] for e in discarded] == [2]
        and [e["path_seq"] for e in held] == [2]
        and discarded[0]["event_seq"] > accepted[1]["event_seq"]
        and [(e["path_seq"], e["goal_status"]) for e in terminals]
        == [(4, 5 if route.get("canceled") else 4)]
        and not any(e["event"] == "test_failed" for e in events)
    )
    expected_gate = validated[0]["gate_id"] if validated else None
    actual_gates = [e["gate_id"] for e in route.get("physical_gate_crossings", [])]
    passed = (route["pass"] and sequence_ok and applied == 2
              and all(w["pass"] for w in transitions) and bool(gate_motion and gate_motion["pass"])
              and actual_gates == [expected_gate] and clearance is not None and clearance >= 0)
    return {"pass": passed, "sequence_verified": sequence_ok, "nav2_applied_replacements": applied,
            "handoff_motion": transitions, "gate_motion": gate_motion,
            "minimum_physx_corridor_clearance_m": clearance, "physical_route": route}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("run", "controller", "layout"):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    result = evaluate(args.run, args.controller, args.layout)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["pass"] else 1)
