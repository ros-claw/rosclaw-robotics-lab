"""Re-evaluate live Luna route selection and independently witnessed gate crossings."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.controller_run import evaluate as evaluate_physics
from evaluator.semantic_probe import evaluate as evaluate_semantics


def evaluate(original, swapped, original_events, swapped_events):
    runs = [original.parents[1], swapped.parents[1]]
    layouts = [Path(json.loads((run / "manifest.json").read_text())["layout"]) for run in runs]
    report = evaluate_semantics(original, swapped, *layouts)
    report["same_nav_parameters"] = (layouts[0] / "nav/params.yaml").read_bytes() == (
        layouts[1] / "nav/params.yaml").read_bytes()
    report["same_map_yaml"] = (layouts[0] / "nav/map.yaml").read_bytes() == (
        layouts[1] / "nav/map.yaml").read_bytes()
    report["motion_authority"] = "locally_validated_luna_intent"
    for run, layout, events, trial in zip(runs, layouts, (original_events, swapped_events),
                                          report["attempts"], strict=True):
        recorded = [json.loads(line) for line in events.read_text().splitlines()]
        live = {event["event"]: event for event in recorded if event["source"] == "luna-live"}
        sequence = ("intent_preflight", "path_validated", "intent_committed",
                    "nav2_accepted", "nav2_result", "odometry_stopped")
        order = [live[kind]["event_seq"] for kind in sequence if kind in live]
        expected = trial["expected_gate"]
        trial["verified_chain"] = (
            len(order) == len(sequence) and order == sorted(set(order))
            and "test_failed" not in live and live["nav2_result"]["goal_status"] == 4
            and all(live[kind].get("gate_id") == expected
                    for kind in ("intent_preflight", "path_validated", "intent_committed"))
        )
        trial["using_normal_camera"] = all(
            json.loads((Path(trial["site"]) / name).read_text()).get("diagnostic_camera_yaw_deg") == 0
            for name in ("request.json", "intent-observation.json"))
        trial["physics"] = evaluate_physics(run, events, layout / "nav/public_scene_manifest.json")
        physical_gates = [g["gate_id"] for g in trial["physics"]["physical_gate_crossings"]]
        report["pass"] &= (trial["verified_chain"] and trial["using_normal_camera"]
                           and trial["physics"]["pass"] and physical_gates == [trial["expected_gate"]])
    report["pass"] &= report["same_nav_parameters"] and report["same_map_yaml"]
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("original", "swapped", "original_events", "swapped_events"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    outcome = evaluate(args.original, args.swapped, args.original_events, args.swapped_events)
    print(json.dumps(outcome, indent=2, ensure_ascii=False))
    raise SystemExit(0 if outcome["pass"] else 1)
