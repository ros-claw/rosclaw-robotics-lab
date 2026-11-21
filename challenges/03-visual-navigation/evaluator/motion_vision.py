"""Audit periodic video decisions and their effect on the live local route check."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.adaptive_run import evaluate as evaluate_adaptation
from evaluator.controller_run import records
from navigation.luna_probe import AUDIT_DECISIONS, parse_intent


def evaluate(run):
    audit = records(run / "motion-audit.jsonl")
    physics = records(run / "physx-trajectory.jsonl")
    returned = {e["model_dir"]: e for e in audit if e["event"] == "audit_returned"}
    trials = []
    for event in audit:
        if event["event"] != "audit_requested":
            continue
        folder = Path(event["model_dir"])
        request = json.loads((folder / "request.json").read_text())
        response = json.loads((folder / "response.json").read_text())
        start, end = (int(response[name]) for name in ("request_host_mono_ns", "received_host_mono_ns"))
        speeds = [math.hypot(*s["linear_angular_velocity"][:2]) for s in physics
                  if start <= int(s["host_mono_ns"]) <= end]
        result = {"observation_id": request["observation_id"], "request_host_mono_ns": start,
                  "latency_s": (end - start) / 1e9,
                  "motion_samples_above_0_05_mps": sum(speed > 0.05 for speed in speeds)}
        if str(folder) in returned:
            raw = json.loads((folder / "api-response.json").read_text())
            intent = parse_intent(raw, request["observation_id"], {}, AUDIT_DECISIONS)
            digest = hashlib.sha256((folder / "model-input.png").read_bytes()).hexdigest()
            accepted = returned[str(folder)]
            result.update({"decision": intent["decision"], "current_goal": accepted["current_goal"],
                           "verified": (digest == request["input_sha256"] == event["image_sha256"]
                                        and raw["model"] == response["model"] == "gpt-6-luna"
                                        and intent == response["intent"]
                                        and accepted["goal_id"] == event["goal_id"])})
        else:
            result.update({"failure": response.get("failure")})
        trials.append(result)
    adopted, early = [], []
    valid_reviews = {trial["observation_id"] for trial in trials
                     if trial.get("decision") == "review" and trial.get("verified")
                     and trial.get("current_goal")}
    for stream in run.glob("controller-*.jsonl"):
        for event in records(stream):
            if event["event"] == "visual_review_adopted":
                adopted.append(event)
            if (event["event"] == "route_blocked" and event.get("visual_review") in valid_reviews
                    and event["standard_blocked"] is False):
                early.append(event)
    repeated = sum(t.get("verified", False) and t["motion_samples_above_0_05_mps"] >= 5
                   for t in trials)
    actors = {e["actor"] for e in records(run / "obstacle-events.jsonl")}
    produced = [t["request_host_mono_ns"] for t in trials]
    adaptation = evaluate_adaptation(run)
    return {"pass": adaptation["pass"] and repeated >= 3 and bool(early) and len(actors) >= 3,
            "audit_requests": len(trials), "audits_during_motion": repeated,
            "median_latency_s": statistics.median(t["latency_s"] for t in trials) if trials else None,
            "median_request_interval_s": statistics.median((b-a)/1e9 for a,b in zip(produced, produced[1:]))
            if len(produced) >= 2 else None,
            "moving_actors": len(actors), "adopted_reviews": adopted, "vision_aided_early_stops": early,
            "trials": trials, "adaptation": adaptation}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = evaluate(args.run.resolve())
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 1)
