"""Validate a dynamic shutter trial directly from both model requests and Isaac state."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.controller_run import evaluate as evaluate_route, records
from navigation.luna_probe import parse_intent


def evaluate(run, session, controller, fixture):
    timeline = records(session)
    requests = [event for event in timeline if event["event"] == "request"]
    replies = [event for event in timeline if event["event"] == "model_returned"]
    if len(requests) < 2 or len(requests) != len(replies):
        return {"pass": False, "reason": "incomplete or single-request vision session"}
    requests_verified = []
    for event, returned in zip(requests, replies, strict=True):
        model = Path(event["model_dir"])
        manifest = json.loads((model / "request.json").read_text())
        reply = json.loads((model / "response.json").read_text())
        digest = hashlib.sha256((model / "model-input.png").read_bytes()).hexdigest()
        raw = json.loads((model / "api-response.json").read_text())
        intent = parse_intent(raw, manifest["observation_id"], manifest["candidate_sides"])
        requests_verified.append({
            "observation_id": manifest["observation_id"],
            "sim_stamp_ns": int(manifest["observation_stamp_ns"]),
            "image_sha256": digest,
            "verified": (digest == event["frame_sha256"] == manifest["input_sha256"]
                         and manifest["observation_id"] == event["observation_id"]
                         and raw["id"] == reply["response_id"]
                         and reply["model"] == raw["model"] == "gpt-6-luna"
                         and reply["intent"] == intent
                         and all(returned.get(field) == value for field, value in intent.items())),
        })
    phases = {event["phase"]: event for event in records(run / "visual-shutter.jsonl")}
    actuation = next(event for event in records(controller) if event["event"] == "follow_path_requested")
    start = int(requests[0]["host_mono_ns"])
    stop = int(actuation["host_mono_ns"])
    physical = [event for event in records(run / "physx-trajectory.jsonl")
                if start <= int(event["host_mono_ns"]) < stop]
    maximum_linear = max(math.hypot(*r["linear_angular_velocity"][:2]) for r in physical)
    maximum_angular = max(abs(r["linear_angular_velocity"][5]) for r in physical)
    stamps = [row["sim_time_s"] for row in physical]
    largest_gap = max(b - a for a, b in zip(stamps, stamps[1:]))
    parked = (maximum_linear <= 0.03 and maximum_angular <= 0.08 and 0 < largest_gap <= 0.15
              and all(row["source"] == "physx" for row in physical))
    actual = evaluate_route(run, controller, fixture / "nav/public_scene_manifest.json")
    expected = json.loads((fixture / "evaluator_manifest.json").read_text())["expected_gate"]
    changed_visibility = (
        len(requests_verified) >= 2
        and requests_verified[0]["sim_stamp_ns"] < round(phases["opening"]["sim_time_s"] * 1e9)
        and requests_verified[-1]["sim_stamp_ns"] >= round(phases["open"]["sim_time_s"] * 1e9)
        and len({item["observation_id"] for item in requests_verified}) == len(requests_verified)
        and requests_verified[0]["image_sha256"] != requests_verified[-1]["image_sha256"]
    )
    decisions = [event["decision"] for event in replies]
    valid = (changed_visibility and all(item["verified"] for item in requests_verified) and parked
             and len(replies) == len(requests) and decisions[0] == "reobserve"
             and all(decision == "reobserve" for decision in decisions[:-1])
             and decisions[-1] == "select_candidate" and replies[-1]["candidate_id"] == expected
             and actual["pass"] and [g["gate_id"] for g in actual["physical_gate_crossings"]] == [expected]
             and int(replies[-1]["host_mono_ns"]) < stop)
    return {"pass": valid, "decisions": decisions, "expected_gate": expected,
            "first_last_visibility_changed": changed_visibility, "api_requests": requests_verified,
            "parked_until_action": parked, "peak_parked_linear_mps": maximum_linear,
            "peak_parked_angular_radps": maximum_angular, "physical_route": actual}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "session", "controller", "fixture"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = evaluate(args.run, args.session, args.controller, args.fixture)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
