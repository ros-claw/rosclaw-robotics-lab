"""Operator-only resettable session fixtures. Never exposed as robot perception."""

import json
from pathlib import Path
from pxr import Gf, UsdGeom, UsdPhysics


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
        records.append({"path": str(p.GetPath()), **box, "collision_enabled": True})
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
