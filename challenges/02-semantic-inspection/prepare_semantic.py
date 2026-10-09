#!/usr/bin/env python3
"""Operator preparation of an immutable generated-target SIM Body before input."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PATROL = HERE.parent / "01-isaac-warehouse-patrol"
sys.path.insert(0, str(PATROL / "rosclaw"))
from prepare import prepare
from propose_observation import target_split


def build_scenario(physics, map_yaml, *, require_lidar=False):
    inventory = physics / "stage-inventory.json"
    data = json.loads(inventory.read_text())
    manifest = target_split(data)
    development = [p for p, s in manifest.items() if s == "development"]
    from propose_observation import free_grid

    _, map_image, _, _ = free_grid(map_yaml)
    params = PATROL / "config/patrol_navigation_params.yaml"
    sources = {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [inventory, map_yaml, map_image, params]
    }
    initial = json.loads((physics / "physics-latest.json").read_text())
    import time

    if (
        not 0 <= time.time() - initial["wall_time"] <= 2
        or not initial.get("timeline_playing")
        or not initial.get("collision_observer_complete")
        or initial.get("collision_count")
        or initial.get("contact_errors")
    ):
        raise ValueError(
            "Fresh playing collision-free initial observation required before Body compilation"
        )
    import math

    if (
        math.hypot(*initial["linear_velocity_xyz"][:2]) > 0.02
        or abs(initial["angular_velocity_xyz"][2]) > 0.1
    ):
        raise ValueError("Robot must be stopped before immutable initial pose binding")
    p = initial["physics_transforms_xyzw"][0]
    x, y, z, w = p[3:7]
    import math

    start = {
        "x": p[0],
        "y": p[1],
        "yaw": math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    }
    contract = {
        "kind": "immutable_semantic_proposal_SIM_v1",
        "frame_id": "map",
        "real_allowed": False,
        "allowed_development_targets": development,
        "source_sha256": sources,
        "initial_observer_id": initial["observer_id"],
        "return_pose": start,
        "proposal_max_wall_age_seconds": 90,
        "proposal_max_sim_age_seconds": 60,
        "footprint_padding_and_margin_m": 0.08,
        "single_goal_timeout_seconds": 420,
        "position_tolerance_m": 0.4,
        "stable_dwell_sim_seconds": 2,
        "maximum_recoveries": 6,
        "required_non_floor_collisions": 0,
        "inspection_scope": "known-region LiDAR observation"
        if require_lidar
        else "semantic observation-point navigation only",
    }
    observation_schema = {"type": "object", "additionalProperties": True}
    return {
        "navigation_contract": contract,
        "config": {
            "scenario": "semantic_observation",
            "verification_schema": "rosclaw.semantic_mission_verification.v1",
            "verification_artifact_name": "mission.verification.json",
            "verification_capability": "mission.verify_and_remember",
            "semantic_source_directory": str(HERE),
            "semantic_contract": contract,
            "semantic_map_yaml": str(map_yaml),
            "initial_physics": initial,
            "expected_order": [],
            "selection_rule": "nearest_development_shelf_west_of_initial_pose",
            "require_target_lidar": require_lidar,
        },
        "mcp": {
            "name": "isaac-semantic-observation",
            "args": [str(HERE / "native_tools.py"), "--directory", "{ROOT}"],
            "observation_tools": ["semantic.observe_candidates"],
            "action_tools": [
                "navigation.navigate_to_pose",
                "mission.verify_and_remember",
            ],
            "output_schemas": {"semantic.observe_candidates": observation_schema},
        },
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--physics", type=Path, required=True)
    p.add_argument("--map-yaml", type=Path, required=True)
    p.add_argument("--task", required=True)
    p.add_argument("--require-target-lidar", action="store_true")
    p.add_argument("--use-configured-model", action="store_true")
    a = p.parse_args()
    root = a.directory.resolve()
    scenario = build_scenario(
        a.physics.resolve(), a.map_yaml.resolve(), require_lidar=a.require_target_lidar
    )
    scenario["mcp"]["args"][-1] = str(root)
    prepare(
        root,
        PATROL,
        a.physics.resolve(),
        a.task,
        [],
        a.use_configured_model,
        scenario=scenario,
    )


if __name__ == "__main__":
    main()
