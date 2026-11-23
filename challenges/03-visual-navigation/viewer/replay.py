"""Build a standalone HTML replay of recorded Luna inputs, Nav2 events and PhysX poses."""

import argparse
import base64
import hashlib
import json
from pathlib import Path


def load(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def available_times(response, events):
    if "received_host_mono_ns" in response:
        return int(response["request_host_mono_ns"]), int(response["received_host_mono_ns"])
    # Older runs first document the model result in the live preflight.
    received = next(e for e in events if e["event"] == "intent_preflight")
    return int(events[0]["host_mono_ns"]), int(received["host_mono_ns"])


def vision_entry(directory, events):
    request = load(directory / "request.json")
    response = load(directory / "response.json")
    image = (directory / "model-input.png").read_bytes()
    if hashlib.sha256(image).hexdigest() != request["input_sha256"]:
        raise ValueError("model input hash mismatch")
    requested, received = available_times(response, events)
    return {"request": request, "response": response, "requested_ns": requested,
            "received_ns": received,
            "image": "data:image/png;base64," + base64.b64encode(image).decode("ascii")}


def bundle(run, model_dir, events_path, layout_path, overview=None, vision_session=None,
           adaptive=False, motion_audit=None):
    session = rows(vision_session) if vision_session else []
    control_files = [events_path] if events_path != Path("-") else []
    if adaptive:
        control_files += [Path(event["controller"]) for event in session if "controller" in event]
        if any(path.resolve().parent != run.resolve() for path in control_files):
            raise ValueError("controller history does not belong to the selected run")
    controller = sorted((dict(event, controller=path.name)
                         for path in dict.fromkeys(control_files) for event in rows(path)),
                        key=lambda event: int(event["host_mono_ns"]))
    events = [event for event in controller
              if event["event"] not in {"twist_command", "odometry_twist"}]
    if any(e["source"] != "luna-live" for e in events):
        raise ValueError("expected a live Luna controller event stream")
    directories = [model_dir]
    if vision_session:
        directories = [Path(event["model_dir"]) for event in session if event["event"] == "request"]
        if (not directories or directories[-1].resolve() != model_dir.resolve()
                or any(path.resolve().parents[1] != run.resolve() for path in directories)):
            raise ValueError("vision session does not belong to the selected run")
    stale_audits = set()
    if motion_audit:
        audit_events = rows(motion_audit)
        stale_audits = {Path(record["model_dir"]) for record in audit_events
                        if record["event"] == "audit_returned" and not record["current_goal"]}
        for record in audit_events:
            if record["event"] == "audit_requested":
                directory = Path(record["model_dir"])
                if directory.resolve().parents[1] != run.resolve():
                    raise ValueError("motion audit does not belong to the selected run")
                if (directory / "response.json").exists():
                    directories.append(directory)
    history = sorted((dict(vision_entry(directory, events), ignored=directory in stale_audits)
                      for directory in directories),
                     key=lambda entry: entry["requested_ns"])
    first_request = history[0]["requested_ns"]
    beginning = min(first_request, int(events[0]["host_mono_ns"])) if events else first_request
    markers = []
    for attempt, entry in enumerate(history, 1):
        is_audit = entry["request"].get("purpose") == "motion_audit" or (
            "review" in entry["request"]["schema"]["schema"]["properties"]["decision"]["enum"])
        for field, event in (("requested_ns", "audit_requested" if is_audit else "model_requested"),
                             ("received_ns", "model_failed" if entry["response"].get("failure")
                              else "audit_ignored" if entry["ignored"]
                              else "audit_returned" if is_audit else "model_received")):
            if field == "requested_ns" and "request_host_mono_ns" not in entry["response"]:
                continue
            markers.append({"event": event, "host_mono_ns": str(entry[field]),
                            "observation": attempt if len(history) > 1 else None})
        entry["requested_seconds"] = (entry["requested_ns"] - beginning) / 1e9
        entry["received_seconds"] = (entry["received_ns"] - beginning) / 1e9
    if adaptive:
        markers += [event for event in session if event["event"] in {
            "physical_stop_verified", "route_recovery", "hold", "route_completed",
        }]
    last_event = max(int(event["host_mono_ns"]) for event in [*events, *markers])
    source = [r for r in rows(run / "physx-trajectory.jsonl")
              if beginning <= int(r["host_mono_ns"]) <= last_event + 3_000_000_000]
    if len(source) < 2 or any(r["source"] != "physx" for r in source):
        raise ValueError("expected a physical trajectory over the recorded episode")
    physical = [[(int(r["host_mono_ns"]) - beginning) / 1e9,
                 *r["pose_xyz_xyzw"][:2]] for r in source]
    paths = {(event["controller"], event["path_seq"]): event["points"] for event in events
             if event["event"] == "path_validated"}
    accepted_paths = [{"path_seq": event["path_seq"],
                       "seconds": (int(event["host_mono_ns"]) - beginning) / 1e9,
                       "points": paths[(event["controller"], event["path_seq"])]}
                      for event in events if event["event"] == "nav2_accepted"]
    rejection = next((e for e in events if e["event"] == "test_failed"), None)
    route_report = model_dir / "physx-evaluation.json"
    timeout_report = run / "timeout-evaluation.json"
    adaptive_report = run / "adaptive-evaluation.json"
    start_pose = physical[0][1:]
    video = None
    if overview:
        first = rows(overview.parent / "frames.jsonl")[0]
        video = {"offset_seconds": (beginning - int(first["host_mono_ns"])) / 1e9,
                 "data": "data:video/mp4;base64," + base64.b64encode(overview.read_bytes()).decode("ascii")}
    return {
        "video": video, "vision": history,
        "layout": load(layout_path),
        "physics": load(route_report) if route_report.exists() else None,
        "timeout": load(timeout_report) if timeout_report.exists() else None,
        "adaptive": load(adaptive_report) if adaptive and adaptive_report.exists() else None,
        "rejection": {"reason": rejection["reason"],
                      "seconds": (int(rejection["host_mono_ns"]) - beginning) / 1e9}
        if rejection else None,
        "events": [{"kind": e["event"], "seconds": (int(e["host_mono_ns"]) - beginning) / 1e9,
                    "path_seq": e.get("path_seq"), "observation": e.get("observation"),
                    "response": e.get("response"), "gate": e.get("gate", e.get("gate_id"))}
                   for e in sorted(events + markers, key=lambda e: int(e["host_mono_ns"]))],
        "trajectory": physical, "accepted_paths": accepted_paths, "start_pose": start_pose,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "model_dir", "events", "layout", "output"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--overview", type=Path)
    parser.add_argument("--vision-session", type=Path)
    parser.add_argument("--adaptive", action="store_true")
    parser.add_argument("--motion-audit", type=Path)
    parser.epilog = "Use - for events in a recorded trial without a controller."
    args = parser.parse_args()
    data = bundle(args.run, args.model_dir, args.events, args.layout, args.overview,
                  args.vision_session, args.adaptive, args.motion_audit)
    encoded = json.dumps(data, ensure_ascii=True, allow_nan=False).replace("<", "\\u003c")
    template = Path(__file__).with_name("replay.html").read_text()
    with args.output.open("x") as stream:
        stream.write(template.replace("__REPLAY_DATA__", encoded, 1))


if __name__ == "__main__":
    main()
