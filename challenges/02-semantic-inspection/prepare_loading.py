#!/usr/bin/env python3
"""Compile separate immutable SIM Body for facility-ground inspection before input."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from prepare_semantic import build_scenario, HERE, PATROL
from loading_geometry import entities
from loading_perception import DEFAULT_THRESHOLDS
from loading_contract import validate_thresholds

sys.path.insert(0, str(PATROL / "rosclaw"))
from prepare import prepare

TASK = "帮我检查一下货架前的装卸区域。重点看看停在货架旁的那辆叉车，以及附近的地面有没有影响通行的障碍物。\n你自己选择合适、安全的观察位置；如果一个位置看不清楚，可以换个位置继续检查。不要冒险进入狭窄区域。\n检查完返回出发点，告诉我检查了哪些地方、发现了什么问题，以及哪些情况还不能确认。"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--physics", type=Path, required=True)
    p.add_argument("--map-yaml", type=Path, required=True)
    p.add_argument("--thresholds", type=Path)
    p.add_argument("--authorized-observation-bounds", type=Path)
    a = p.parse_args()
    root = a.directory.resolve()
    physics = a.physics.resolve()
    scenario = build_scenario(physics, a.map_yaml.resolve())
    prior_path = physics / "loading-known-prior.json"
    if not prior_path.exists():
        raise ValueError("Known facility prior must precede hidden fixture additions")
    prior = json.loads(prior_path.read_text())
    if prior["up_axis"] != "Z" or prior["meters_per_unit"] != 1:
        raise ValueError("Unexpected prior units/frame")
    thresholds = (
        json.loads(a.thresholds.read_text()) if a.thresholds else DEFAULT_THRESHOLDS
    )
    thresholds = validate_thresholds(thresholds)
    contract = scenario["navigation_contract"]
    del contract["allowed_development_targets"]
    contract.update(
        kind="immutable_loading_proposal_SIM_v1",
        allowed_targets=[
            o["path"] for o in entities(prior) if o["category"] == "forklift"
        ],
    )
    contract["source_sha256"][str(prior_path)] = hashlib.sha256(
        prior_path.read_bytes()
    ).hexdigest()
    nav_host = os.environ.get("ROSCLAW_LOADING_NAV_HOST_PARAMS")
    if nav_host:
        nav_path = Path(nav_host)
        contract["source_sha256"][str(nav_path)] = hashlib.sha256(
            nav_path.read_bytes()
        ).hexdigest()
    loading = {
        "known_prior_source": str(prior_path),
        "thresholds": thresholds,
        "relation": "forklift near shelf from measured facility geometry",
        "height_scope": "ground 0±.08m; actual obstacles .10–.80m; no smaller-object clearance claim",
        "real_allowed": False,
    }
    if a.authorized_observation_bounds:
        loading["allowed_observation_bounds"] = json.loads(
            a.authorized_observation_bounds.read_text()
        )
    contract["loading_contract"] = loading
    contract["inspection_scope"] = "loading-area measured ground/height-band passage"
    scenario["config"].update(
        scenario="loading_inspection",
        selection_rule="forklift_near_shelf_geometric_relation",
        loading_contract=loading,
        execution_root=str(root),
        require_target_lidar=False,
    )
    scenario["mcp"].update(
        name="warehouse-loading-inspection",
        args=[str(HERE / "loading_tools.py"), "--directory", str(root)],
        observation_tools=["loading.observe_scene"],
        output_schemas={
            "loading.observe_scene": {"type": "object", "additionalProperties": True}
        },
    )
    scenario["raw_pointcloud"] = True
    prepare(root, PATROL, physics, TASK, [], True, scenario=scenario)
    import shutil

    shutil.copyfile(prior_path, root / "loading-known-prior.json")


if __name__ == "__main__":
    main()
