"""Recompute the park-and-recover sequence from independent live evidence."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.controller_run import evaluate as verify_route, recovery_stop_verified, records
from navigation.candidates import crossings
from navigation.luna_probe import parse_intent


def host_time(event):
    return int(event["host_mono_ns"])


def verify_request(event):
    directory = Path(event["model_dir"])
    request = json.loads((directory / "request.json").read_text())
    raw = json.loads((directory / "api-response.json").read_text())
    response = json.loads((directory / "response.json").read_text())
    digest = hashlib.sha256((directory / "model-input.png").read_bytes()).hexdigest()
    intent = parse_intent(raw, request["observation_id"], request["candidate_sides"])
    return (raw["model"] == response["model"] == "gpt-6-luna"
            and event["frame_sha256"] == request["input_sha256"] == digest
            and intent == response["intent"] and intent["observation_id"] == event["observation_id"])


def yaw(pose):
    _, _, _, x, y, z, w = pose
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def alignment_checks(controller, samples):
    checks = []
    for _, events in controller:
        start = next((e for e in events if e["event"] == "alignment_requested"), None)
        if start is None:
            continue
        finish = next((e for e in events if e["event"] == "alignment_completed"), None)
        if finish is None:
            checks.append({"pass": False, "reason": "alignment did not finish"})
            continue
        a, b = (min(samples, key=lambda s: abs(host_time(s) - host_time(event)))["pose_xyz_xyzw"]
                for event in (start, finish))
        drift = math.dist(a[:2], b[:2])
        delta = math.remainder(yaw(b) - yaw(a), math.tau)
        error = abs(math.remainder(delta - start["turn_rad"], math.tau))
        checks.append({"pass": drift <= 0.15 and error <= 0.38,
                       "physical_translation_m": drift, "requested_rad": start["turn_rad"],
                       "physics_turn_rad": delta})
    return checks


def evaluate(run):
    session = records(next(run.glob("luna-session-*.jsonl")))
    layout = Path(json.loads((run / "manifest.json").read_text())["layout"]) / "nav/public_scene_manifest.json"
    layout_data = json.loads(layout.read_text())
    requests = [event for event in session if event["event"] == "request"]
    verified_stops = [event for event in session if event["event"] == "physical_stop_verified"]
    recovery = [event for event in session if event["event"] == "route_recovery"]
    if not requests or not verified_stops or not recovery:
        return {"pass": False, "reason": "missing live request, physical stop, or recovery evidence"}
    controller = [(path, records(path)) for path in run.glob("controller-*.jsonl")]
    controller.sort(key=lambda item: host_time(item[1][0]))
    actions = [event for _, events in controller for event in events
               if event["event"] == "follow_path_requested"]
    blocked = [event for _, events in controller for event in events
               if event["event"] == "route_blocked" and event.get("source_grid") == "global_costmap"]
    # Every interrupted goal has a verified physical stop before any later submission.
    stops_ok = True
    for event in verified_stops:
        stop_path = Path(event["controller"])
        submitted = next(e for p, events in controller if p == stop_path for e in events
                         if e["event"] == "follow_path_requested")
        later_actions = [action for action in actions if host_time(action) > host_time(submitted)]
        stops_ok &= (recovery_stop_verified(run, stop_path)
                     and all(host_time(event) < host_time(action) for action in later_actions))
    samples = records(run / "physx-trajectory.jsonl")
    start = host_time(actions[0])
    points = [row["pose_xyz_xyzw"][:2] for row in samples if host_time(row) >= start]
    actual_gates = [gate["gate_id"] for gate in layout_data["gates"]
                    for forward, y in crossings(points, gate["cross_x"])
                    if forward and gate["lower_y"] <= y <= gate["upper_y"]]
    final = session[-1]
    route = None
    if final["event"] == "route_completed":
        route = verify_route(run, Path(final["controller"]), layout)
        kind = "local_resume" if final["route_version"] == 1 else "alternate"
        choices_ok = (route["pass"] and actual_gates == [final["gate"]]
                      and (len(requests) == 1 if kind == "local_resume" else
                           len(requests) >= 2 and recovery[0]["gate"] in requests[-1]["excluded"]
                           and host_time(requests[-1]) > host_time(verified_stops[-1])
                           and requests[0]["observation_id"] != requests[-1]["observation_id"]))
    else:
        kind = "hold"
        tail = [row for row in samples if host_time(row) >= host_time(final)]
        choices_ok = (final["event"] == "hold" and final.get("reason") == "committed_route_unavailable"
                      and not actual_gates and len(actions) == 1 and len(requests) == 1 and bool(tail)
                      and (host_time(tail[-1]) - host_time(final)) / 1e9 >= 2
                      and max(math.hypot(*row["linear_angular_velocity"][:2]) for row in tail) <= .03)
    alignments = alignment_checks(controller, samples)
    frames_verified = all(verify_request(event) for event in requests)
    no_contacts = not any(event["classification"] == "non_floor"
                          for event in records(run / "physx-contacts.jsonl"))
    return {"pass": bool(blocked and stops_ok and choices_ok and no_contacts and frames_verified
                         and all(check["pass"] for check in alignments)),
            "outcome": kind, "model_requests": len(requests), "follow_paths": len(actions),
            "model_frames_verified": frames_verified, "alignments": alignments,
            "sensor_trigger": blocked, "verified_stops": len(verified_stops),
            "stop_before_resume": stops_ok, "physical_gates": actual_gates,
            "no_non_floor_contacts": no_contacts, "final_route": route}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = evaluate(args.run.resolve())
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 1)
