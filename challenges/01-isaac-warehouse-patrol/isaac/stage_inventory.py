"""Read authored/composed USD geometry for scene and Body provenance."""

from pxr import Gf, Usd, UsdGeom, UsdPhysics


def inspect_scene(stage):
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    floors, objects, joints, colliders = {}, [], [], []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        robot = path.startswith("/World/Nova_Carter_ROS/")
        name = prim.GetName().lower()
        floor_candidate = "groundplane" in path.lower() and "collisionplane" in name
        scene_object = (
            not robot
            and len(prim.GetPath().GetPrefixes()) <= 7
            and any(
                word in name
                for word in [
                    "shelf",
                    "rack",
                    "door",
                    "forklift",
                    "entrance",
                    "wall",
                    "gate",
                ]
            )
        )
        if floor_candidate and prim.GetTypeName() == "Plane":
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            axis = str(prim.GetAttribute("axis").Get() or "Z")
            normal = matrix.TransformDir(
                {
                    "X": Gf.Vec3d(1, 0, 0),
                    "Y": Gf.Vec3d(0, 1, 0),
                    "Z": Gf.Vec3d(0, 0, 1),
                }[axis]
            ).GetNormalized()
            position = matrix.ExtractTranslation()
            enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            floors[path] = {
                "path": path,
                "type": "Plane",
                "normal": list(normal),
                "position": list(position),
                "collision_enabled": enabled,
                "verified_horizontal_floor": abs(normal[2]) > 0.9999
                and abs(position[2]) < 0.05
                and enabled is not False,
            }
            continue
        if (
            floor_candidate
            or scene_object
            or (robot and prim.HasAPI(UsdPhysics.CollisionAPI))
        ):
            try:
                box = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                if box.IsEmpty():
                    continue
                minimum, maximum = list(box.GetMin()), list(box.GetMax())
                row = {
                    "path": path,
                    "type": prim.GetTypeName(),
                    "min": minimum,
                    "max": maximum,
                }
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    row["collision_enabled"] = (
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                    )
                if floor_candidate:
                    horizontal = maximum[2] - minimum[2] < 0.002
                    at_ground = abs(maximum[2]) < 0.05
                    row["verified_horizontal_floor"] = (
                        horizontal
                        and at_ground
                        and row.get("collision_enabled") is not False
                    )
                    floors[path] = row
                if scene_object:
                    objects.append(row)
                if robot:
                    colliders.append(row)
            except Exception as exc:
                if floor_candidate:
                    floors[path] = {
                        "error": repr(exc),
                        "verified_horizontal_floor": False,
                    }
        if robot and prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joints.append(
                {
                    "name": prim.GetName(),
                    "path": path,
                    "type": prim.GetTypeName(),
                    "body0": [str(p) for p in joint.GetBody0Rel().GetTargets()],
                    "body1": [str(p) for p in joint.GetBody1Rel().GetTargets()],
                    "attributes": {
                        a.GetName(): str(a.Get())
                        for a in prim.GetAttributes()
                        if a.HasAuthoredValue()
                    },
                }
            )
    return {
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "floors": floors,
        "scene_objects": objects,
        "robot_colliders": colliders,
        "robot_joints": joints,
    }
