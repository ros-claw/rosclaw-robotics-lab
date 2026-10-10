"""Operator-only resettable session fixtures. Never exposed as robot perception."""

import json
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def apply_fixture(stage, config_path, output):
    config = json.loads(Path(config_path).read_text())
    prior = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    records = []
    for i, box in enumerate(config.get("boxes", [])):
        p = UsdGeom.Cube.Define(stage, f"/World/ROSClawLoadingFixture/box_{i}")
        p.CreateSizeAttr(1.0)
        p.AddTranslateOp().Set(Gf.Vec3d(*box["center"]))
        p.AddScaleOp().Set(Gf.Vec3d(*box["size"]))
        p.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.27, 0.08)])
        UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        bounds = (
            UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                ["default", "render", "proxy", "guide"],
                False,
                True,
            )
            .ComputeWorldBound(p.GetPrim())
            .ComputeAlignedRange()
        )
        records.append(
            {
                "path": str(p.GetPath()),
                **box,
                "collision_enabled": bool(
                    UsdPhysics.CollisionAPI(p.GetPrim()).GetCollisionEnabledAttr().Get()
                ),
                "bounds": {"min": list(bounds.GetMin()), "max": list(bounds.GetMax())},
            }
        )
    if "initial_pose" in config:
        x, y, yaw = config["initial_pose"]
        # Reset-time transform only, before play. No ROS/actuator motion command.
        root = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Nova_Carter_ROS"))
        matrix = Gf.Matrix4d(1.0)
        matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw * 180 / 3.141592653589793))
        matrix.SetTranslateOnly(Gf.Vec3d(x, y, 0))
        root.MakeMatrixXform().Set(matrix)
    stage.SetEditTarget(prior)
    (output / "loading-fixture-truth.json").write_text(
        json.dumps(
            {
                "seed": config.get("seed"),
                "case": config.get("case"),
                "boxes": records,
                "initial_pose": config.get("initial_pose"),
                "role": "independent evaluator truth; never perception",
            },
            indent=2,
        )
        + "\n"
    )


def measure_loading_truth(stage, output):
    """Read-only evaluator PhysX queries. Never copied into Native's input view."""
    import carb
    import math
    import numpy as np
    from omni.physx import get_physx_scene_query_interface
    from loading_geometry import entities, relations, region_for_relation

    prior = json.loads((output / "loading-known-prior.json").read_text())
    relation = relations(entities(prior))
    region = region_for_relation(relation["ranked"][0])
    res, radius = 0.2, math.ceil((0.75 + 0.2 / math.sqrt(2)) / 0.2)
    size = np.ceil((np.array(region["max"]) - region["min"]) / res).astype(int)
    query = get_physx_scene_query_interface()
    cells = []
    for y in range(int(size[1])):
        for x in range(int(size[0])):
            cx, cy = np.array(region["min"]) + (np.array([x, y]) + 0.5) * res
            hits = []

            def found(hit):
                path = str(hit.collision)
                if not path.startswith("/World/Nova_Carter_ROS/"):
                    hits.append(path)
                return True

            query.overlap_box(
                carb.Float3((radius + 0.5) * res, (radius + 0.5) * res, 0.35),
                carb.Float3(float(cx), float(cy), 0.45),
                carb.Float4(0, 0, 0, 1),
                found,
                False,
            )
            cells.append({"cell": [x, y], "colliders": sorted(set(hits))})
    (output / "loading-physx-truth.json").write_text(
        json.dumps(
            {
                "role": "Independent PhysX configuration-space oracle; not robot perception",
                "region": region,
                "resolution_m": res,
                "clearance_cells": radius,
                "query_half_extent_xy_m": (radius + 0.5) * res,
                "height_scope_m": [0.1, 0.8],
                "size": size.tolist(),
                "cells": cells,
                "relation": {
                    "target": relation["ranked"][0]["target"]["path"],
                    "distances_m": [r["shelf_distance_m"] for r in relation["ranked"]],
                },
                "api_reference": "https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physx/docs/api/python.html",
            },
            indent=2,
        )
        + "\n"
    )
