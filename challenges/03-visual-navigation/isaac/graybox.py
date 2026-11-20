"""Replace the warehouse environment inside the existing Nova Carter world."""

import json
from pathlib import Path

from pxr import Gf, Usd, UsdGeom


def apply_graybox(stage, directory):
    x, y, yaw = json.loads((directory / "spawn.json").read_text())
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in ("/World/warehouse_with_forklifts", "/World/warehouse_extras"):
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
        room = UsdGeom.Xform.Define(stage, "/World/LunaGraybox").GetPrim()
        if not room.GetReferences().AddReference(str(directory / "layout.usda"), "/World"):
            raise RuntimeError("Failed to compose the SimScene graybox")
        robot = stage.GetPrimAtPath("/World/Nova_Carter_ROS")
        robot.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, 0.0))
        rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw * 180 / 3.141592653589793)
        robot.GetAttribute("xformOp:orient").Set(rotation.GetQuat())
        child = stage.GetPrimAtPath("/World/LunaGraybox/floor")
        if not child.IsValid():
            raise RuntimeError("SimScene floor did not compose")
