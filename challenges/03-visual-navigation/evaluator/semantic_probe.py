"""Evaluate frozen Luna replies against private, paired fixture truth."""

import argparse
import hashlib
import json
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text())


def evaluate(original, swapped, original_layout, swapped_layout):
    attempts = []
    for path, layout in ((original, original_layout), (swapped, swapped_layout)):
        request = read_json(path / "request.json")
        response = read_json(path / "response.json")
        truth = read_json(layout / "evaluator_manifest.json")
        intent = response.get("intent", {})
        image_hash = hashlib.sha256((path / "model-input.png").read_bytes()).hexdigest()
        attempts.append({
            "variant": truth["variant"], "expected_gate": truth["expected_gate"],
            "observed_gate": intent.get("candidate_id"), "decision": intent.get("decision"),
            "matched": intent.get("decision") == "select_candidate"
            and intent.get("candidate_id") == truth["expected_gate"],
            "image_sha256": image_hash, "hash_verified": request["input_sha256"] == image_hash,
            "model": response.get("model"), "latency_seconds": response.get("elapsed_seconds"),
            "site": str(path),
        })
    left = read_json(original / "request.json")
    right = read_json(swapped / "request.json")
    same_prompt = all(left[key] == right[key] for key in (
        "instructions", "task", "candidate_sides", "model_requested",
    ))
    same_geometry = all((original_layout / "nav" / file).read_bytes()
                        == (swapped_layout / "nav" / file).read_bytes()
                        for file in ("public_scene_manifest.json", "map.pgm"))
    changed_gate = attempts[0]["observed_gate"] != attempts[1]["observed_gate"]
    changed_rgb = attempts[0]["image_sha256"] != attempts[1]["image_sha256"]
    passed = (same_prompt and same_geometry and changed_rgb and changed_gate
              and all(a["matched"] and a["hash_verified"] for a in attempts))
    return {"pass": passed, "same_neutral_prompt": same_prompt, "same_public_geometry": same_geometry,
            "changed_rgb": changed_rgb, "changed_gate": changed_gate, "motion_authority": False,
            "attempts": attempts}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("original", "swapped", "original_layout", "swapped_layout"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report = evaluate(args.original, args.swapped, args.original_layout, args.swapped_layout)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    raise SystemExit(0 if report["pass"] else 1)
