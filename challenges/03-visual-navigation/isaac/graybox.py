"""Replace the warehouse environment inside the existing Nova Carter world."""

import json
import math

from pxr import Gf, Usd, UsdGeom, UsdPhysics


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
        rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))
        robot.GetAttribute("xformOp:orient").Set(rotation.GetQuat())
        child = stage.GetPrimAtPath("/World/LunaGraybox/floor")
        if not child.IsValid():
            raise RuntimeError("SimScene floor did not compose")
        # A box floor destabilized the stock cylindrical wheel contacts.
        # Use the same analytic Z-plane as the standalone Nova Carter asset.
        if child.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
        plane = UsdGeom.Plane.Define(stage, "/World/LunaGraybox/physics_floor")
        plane.CreateAxisAttr("Z")
        plane.CreatePurposeAttr("guide")
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())


def configure_final_velocity(stage):
    matches = [prim for prim in stage.Traverse()
               if prim.HasAttribute("node:type")
               and prim.GetAttribute("node:type").Get() == "isaacsim.ros2.bridge.ROS2SubscribeTwist"]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one Nova Carter Twist subscriber, found {len(matches)}")
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        matches[0].GetAttribute("inputs:topicName").Set("/luna/cmd_vel")
    print(f"Final simulated base Twist subscriber is {matches[0].GetPath()} at /luna/cmd_vel", flush=True)
