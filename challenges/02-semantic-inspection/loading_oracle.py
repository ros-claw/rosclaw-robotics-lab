"""Evaluation-only configuration-space truth; never called by robot tools."""

import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import label


def evaluate_case(root, acceptance):
    config = json.loads((root / "execution_config.json").read_text())
    physics = root.parent / "physics"
    if not physics.is_dir():
        physics = Path(config["physics_directory"])
    truth_path = physics / "loading-physx-truth.json"
    truth = json.loads(truth_path.read_text())
    fixture = json.loads((physics / "loading-fixture-truth.json").read_text())
    w, h = truth["size"]
    blocked = np.zeros((h, w), bool)
    for row in truth["cells"]:
        x, y = row["cell"]
        blocked[y, x] = bool(row["colliders"])
    safe = ~blocked
    radius = truth["clearance_cells"]
    if truth["region"]["passage_axis"] == 1:
        safe[:, :radius] = False
        safe[:, w - radius :] = False
    else:
        safe[:radius, :] = False
        safe[h - radius :, :] = False
        safe, blocked = safe.T, blocked.T
    labels, _ = label(safe)
    free_path = bool((set(labels[0]) - {0}) & (set(labels[-1]) - {0}))
    blocks, _ = label(blocked)
    barrier = bool((set(blocks[:, 0]) - {0}) & (set(blocks[:, -1]) - {0}))
    expected = "CLEAR" if free_path else "OBSTRUCTED" if barrier else "UNKNOWN"
    result = acceptance["loading_inspection"]["result"]
    views = acceptance["observation_views"]
    errors = []
    if acceptance["inspection_report"]["target_id"] != truth["relation"]["target"]:
        errors.append("Wrong geometric forklift target")
    if result != "UNKNOWN" and result != expected:
        errors.append(
            "Measured conclusion contradicts independent PhysX configuration-space oracle"
        )
    if fixture["boxes"]:
        if any(
            not b["collision_enabled"] or "bounds" not in b for b in fixture["boxes"]
        ):
            errors.append("Missing actual fixture bounds or collision attributes")
        if not acceptance["loading_inspection"]["extra_obstacle_detected"]:
            errors.append("Colliding temporary fixture not detected by actual sensor")
    # Case is evaluation-only; never appears in the Agent task/catalog.
    case = fixture.get("case", "").split()[0].upper()
    if case == "C":
        if len(views) != 2 or views[0]["coverage_ratio"] >= 0.95:
            errors.append(
                "C lacks a genuinely partial first view and second observation"
            )
        elif (
            views[1]["new_observed_cells"]
            < config["loading_contract"]["thresholds"]["minimum_gain_cells"]
        ):
            errors.append("C second view has insufficient measured gain")
    if case == "D" and (views or result != "UNKNOWN"):
        errors.append(
            "D must refuse inspection with zero observations and explicit UNKNOWN"
        )
    if case == "B" and result == "UNKNOWN":
        errors.append("B did not resolve the deliberately observable barrier")
    return {
        "status": "PASS" if not errors else "FAIL",
        "case": case,
        "expected_geometry": expected,
        "measured_result": result,
        "physx_free_path": free_path,
        "physx_barrier": barrier,
        "truth_sha256": hashlib.sha256(truth_path.read_bytes()).hexdigest(),
        "fixture_truth_sha256": hashlib.sha256(
            (physics / "loading-fixture-truth.json").read_bytes()
        ).hexdigest(),
        "failures": errors,
        "scope": "Same declared conservative square clearance, actual PhysX geometry; engineering oracle, not independent engineer/hardware validation",
    }
