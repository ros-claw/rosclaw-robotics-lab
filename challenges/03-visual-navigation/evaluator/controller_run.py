"""Audit physical gate crossing and stopping using separate Isaac PhysX records."""

import argparse
import json
import math
from pathlib import Path


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def evaluate(run, events_path, public_scene):
    layout = json.loads(public_scene.read_text())
    events = records(events_path)
    accepted = next((event for event in events if event["event"] == "nav2_accepted"), None)
    path = next((event for event in events if event["event"] == "path_validated"), None)
    stop = next((event for event in events if event["event"] == "odometry_stopped"), None)
    if accepted is None or path is None or stop is None:
        return {"pass": False, "reason": "motion and odometry stop events are incomplete"}
    begin = int(accepted["host_mono_ns"])
    finish = int(stop["host_mono_ns"])
    trajectory = [row for row in records(run / "physx-trajectory.jsonl")
                  if begin <= int(row["host_mono_ns"]) <= finish]
    if len(trajectory) < 2 or any(row["source"] != "physx" for row in trajectory):
        return {"pass": False, "reason": "physical trajectory is missing from the action interval"}
    crossed = []
    coordinates = [row["pose_xyz_xyzw"][:2] for row in trajectory]
    for gate in layout["gates"]:
        x_gate = gate["cross_x"]
        for a, b in zip(coordinates, coordinates[1:]):
            if not a[0] < x_gate <= b[0]:
                continue
            fraction = (x_gate - a[0]) / (b[0] - a[0])
            y = a[1] + fraction * (b[1] - a[1])
            if gate["lower_y"] <= y <= gate["upper_y"]:
                crossed.append({"gate_id": gate["gate_id"], "xy_m": [x_gate, y]})
    tail = [row for row in trajectory if row["sim_time_s"] >= trajectory[-1]["sim_time_s"] - 0.75]
    final_speed = max(math.hypot(*row["linear_angular_velocity"][:2]) for row in tail)
    dwell = tail[-1]["sim_time_s"] - tail[0]["sim_time_s"]
    stopped = (dwell >= 0.65 and final_speed <= 0.03
               and max(abs(row["linear_angular_velocity"][5]) for row in tail) <= 0.08)
    contacts = records(run / "physx-contacts.jsonl")
    floor_witnessed = any(row["classification"] == "floor" for row in contacts)
    non_floor = [row for row in contacts if row["classification"] == "non_floor"]
    distance = math.dist(coordinates[-1], layout["terminal_xy"])
    canceled = any(event["event"] == "cancel_acknowledged" for event in events)
    movement = sum(math.dist(a, b) for a, b in zip(coordinates, coordinates[1:]))
    if canceled:
        outcome = movement >= 0.10 and all(row["gate_id"] == path["gate_id"] for row in crossed)
    else:
        outcome = [row["gate_id"] for row in crossed] == [path["gate_id"]] and distance <= 0.45
    return {
        "pass": outcome and stopped and floor_witnessed and not non_floor,
        "source": "physx", "samples": len(trajectory), "canceled": canceled,
        "physical_gate_crossings": crossed, "physical_distance_to_terminal_m": distance,
        "physical_distance_traveled_m": movement, "last_dwell_sim_s": dwell,
        "max_speed_during_dwell_mps": final_speed, "physx_stopped": stopped,
        "floor_contact_witnessed": floor_witnessed, "non_floor_contact_pairs": non_floor,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("events", type=Path)
    parser.add_argument("public_scene", type=Path)
    args = parser.parse_args()
    result = evaluate(args.run, args.events, args.public_scene)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["pass"] else 1)
