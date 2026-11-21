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


def bundle(run, model_dir, events_path, layout_path, overview=None):
    request = load(model_dir / "request.json")
    response = load(model_dir / "response.json")
    image = (model_dir / "model-input.png").read_bytes()
    if hashlib.sha256(image).hexdigest() != request["input_sha256"]:
        raise ValueError("model input hash mismatch")
    controller = rows(events_path) if events_path != Path("-") else []
    events = [event for event in controller
              if event["event"] not in {"twist_command", "odometry_twist"}]
    if any(e["source"] != "luna-live" for e in events):
        raise ValueError("expected a live Luna controller event stream")
    request_ns, received_ns = available_times(response, events)
    beginning = min(request_ns, int(events[0]["host_mono_ns"])) if events else request_ns
    outcome_kind = "model_failed" if response.get("failure") else "model_received"
    markers = [{"event": outcome_kind, "host_mono_ns": str(received_ns)}]
    if "request_host_mono_ns" in response:
        markers.append({"event": "model_requested", "host_mono_ns": str(request_ns)})
    last_event = int(events[-1]["host_mono_ns"]) if events else received_ns
    source = [r for r in rows(run / "physx-trajectory.jsonl")
              if beginning <= int(r["host_mono_ns"]) <= last_event + 3_000_000_000]
    if len(source) < 2 or any(r["source"] != "physx" for r in source):
        raise ValueError("expected a physical trajectory over the recorded episode")
    physical = [[(int(r["host_mono_ns"]) - beginning) / 1e9,
                 *r["pose_xyz_xyzw"][:2]] for r in source]
    paths = {event["path_seq"]: event["points"] for event in events
             if event["event"] == "path_validated"}
    accepted_paths = [{"path_seq": event["path_seq"],
                       "seconds": (int(event["host_mono_ns"]) - beginning) / 1e9,
                       "points": paths[event["path_seq"]]}
                      for event in events if event["event"] == "nav2_accepted"]
    rejection = next((e for e in events if e["event"] == "test_failed"), None)
    route_report = model_dir / "physx-evaluation.json"
    timeout_report = run / "timeout-evaluation.json"
    start_pose = physical[0][1:]
    video = None
    if overview:
        first = rows(overview.parent / "frames.jsonl")[0]
        video = {"offset_seconds": (beginning - int(first["host_mono_ns"])) / 1e9,
                 "data": "data:video/mp4;base64," + base64.b64encode(overview.read_bytes()).decode("ascii")}
    return {
        "video": video, "request": request, "response": response,
        "image": "data:image/png;base64," + base64.b64encode(image).decode("ascii"),
        "layout": load(layout_path),
        "physics": load(route_report) if route_report.exists() else None,
        "timeout": load(timeout_report) if timeout_report.exists() else None,
        "rejection": {"reason": rejection["reason"],
                      "seconds": (int(rejection["host_mono_ns"]) - beginning) / 1e9}
        if rejection else None,
        "model_received_seconds": (received_ns - beginning) / 1e9,
        "events": [{"kind": e["event"], "seconds": (int(e["host_mono_ns"]) - beginning) / 1e9,
                    "path_seq": e.get("path_seq")}
                   for e in sorted(events + markers, key=lambda e: int(e["host_mono_ns"]))],
        "trajectory": physical, "accepted_paths": accepted_paths, "start_pose": start_pose,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "model_dir", "events", "layout", "output"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--overview", type=Path)
    parser.epilog = "Use - for events in a recorded trial without a controller."
    args = parser.parse_args()
    data = bundle(args.run, args.model_dir, args.events, args.layout, args.overview)
    encoded = json.dumps(data, ensure_ascii=True, allow_nan=False).replace("<", "\\u003c")
    template = Path(__file__).with_name("replay.html").read_text()
    with args.output.open("x") as stream:
        stream.write(template.replace("__REPLAY_DATA__", encoded, 1))


if __name__ == "__main__":
    main()
