"""Compare route timings and physically witnessed braking for two completed trials."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import median, quantiles
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.adaptive_run import evaluate as adaptive_result
from evaluator.controller_run import first_settled_index, records


def host_ns(record):
    return int(record["host_mono_ns"])


def pose_xy(record):
    return record["pose_xyz_xyzw"][:2]


def evaluate(run):
    physical = records(run / "physx-trajectory.jsonl")
    session = records(next(run.glob("luna-session-*.jsonl")))
    reviews = records(run / "motion-audit.jsonl")
    first = next(e for e in session if e["event"] == "request")
    complete = next(e for e in session if e["event"] == "route_completed")
    commands = [records(path) for path in run.glob("controller-*.jsonl")]
    by_goal = []
    braking = []
    for events in commands:
        initial = next((e for e in events if e["event"] == "nav2_accepted"), None)
        terminal = next((e for e in events if e["event"] == "nav2_result"), None)
        if initial is None or terminal is None:
            continue
        wall_s = (host_ns(terminal) - host_ns(initial)) / 1e9
        sim_s = (int(terminal["sim_stamp_ns"]) - int(initial["sim_stamp_ns"])) / 1e9
        by_goal.append((wall_s, sim_s))
        blocked = next((e for e in events if e["event"] == "route_blocked"), None)
        if blocked is None or host_ns(blocked) > host_ns(terminal):
            continue
        reported = next(e for e in events if e["event"] == "odometry_stopped")
        braking_samples = [p for p in physical
                           if host_ns(blocked) <= host_ns(p) <= host_ns(reported) + 3_000_000_000]
        settled = first_settled_index(braking_samples, host_ns(reported))
        if settled is None:
            raise ValueError("stop cannot be independently quantified")
        distance = sum(math.dist(pose_xy(a), pose_xy(b))
                       for a,b in zip(braking_samples[:settled[0]], braking_samples[1:settled[0]+1]))
        braking.append({"detection_to_low_speed_wall_s":
                        (host_ns(braking_samples[settled[0]]) - host_ns(blocked)) / 1e9,
                        "distance_until_low_speed_m": distance})
    final_route_result = adaptive_result(run)
    wall, sim = (sum(values) for values in zip(*by_goal))
    params = yaml.safe_load((run / "nav2-runtime.yaml").read_text())
    vmax = params["controller_server"]["ros__parameters"]["FollowPath"]["vx_max"]
    speeds = [math.hypot(*s["linear_angular_velocity"][:2]) for s in physical
              if host_ns(first) <= host_ns(s) <= host_ns(complete)]
    audit_inputs = [row for row in reviews if row["event"] == "audit_requested"]
    latencies = [json.loads((Path(r["model_dir"]) / "response.json").read_text())["elapsed_seconds"]
                 for r in audit_inputs]
    interval = [(host_ns(b) - host_ns(a))/1e9 for a,b in zip(audit_inputs, audit_inputs[1:])]
    return {"run": run.name, "physics_pass": final_route_result["pass"],
            "physical_gates": final_route_result["physical_gates"],
            "collision_free": final_route_result["no_non_floor_contacts"],
            "terminal_distance_m": final_route_result["final_route"]["physical_distance_to_terminal_m"],
            "mpc_linear_cap_mps": vmax, "action_wall_s": wall, "action_sim_s": sim,
            "active_real_time_factor": sim / wall,
            "system_wall_s": (host_ns(complete) - host_ns(first)) / 1e9,
            "peak_speed_mps": max(speeds), "p95_speed_mps": quantiles(speeds, n=20)[18],
            "braking": braking, "motion_audit_latency_median_s": median(latencies),
            "motion_audit_interval_median_s": median(interval) if interval else None}


def fixture_identity(run):
    manifest = json.loads((run / "manifest.json").read_text())
    layout = Path(manifest["layout"])
    digest = hashlib.sha256()
    for name in ("nav/public_scene_manifest.json", "nav/map.pgm", "sim/layout.usda"):
        payload = (layout / name).read_bytes()
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    return digest.hexdigest(), "--record-overview" in manifest["commands"][0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    baseline, trial = args.baseline.resolve(), args.trial.resolve()
    same_scene = (baseline / "obstacle-recipe.json").read_bytes() == (trial / "obstacle-recipe.json").read_bytes()
    same_fixture = fixture_identity(baseline) == fixture_identity(trial)
    left, right = evaluate(baseline), evaluate(trial)
    report = {"same_obstacle_recipe": same_scene,
              "same_geometry_and_recording_mode": same_fixture,
              "passed": same_scene and same_fixture and left["physics_pass"] and right["physics_pass"],
              "baseline": left, "trial": right,
              "active_wall_reduction_percent": (1 - right["action_wall_s"]/left["action_wall_s"]) * 100,
              "task_wall_reduction_percent": (1 - right["system_wall_s"]/left["system_wall_s"]) * 100}
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
