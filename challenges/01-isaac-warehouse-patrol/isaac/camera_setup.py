"""Display-only cameras: no collisions, sensor replacement or physics changes."""

import math
from pxr import Gf, UsdGeom


def setup_camera(stage, view):
    from omni.kit.viewport.utility import get_active_viewport

    poses = {
        "overview": ((6, -8, 4.8), (0, 4, 0), (0, 0, 1)),
        "top": ((0, 1, 5.5), (0, 1, 0), (1, 0, 0)),
        "follow": ((-2, -1, 3), (-6, -1, 0.6), (0, 0, 1)),
    }
    if view not in poses:
        return lambda transform: None
    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    camera = UsdGeom.Camera.Define(stage, "/World/ROSClawDisplayCamera")
    camera.CreateFocalLengthAttr(12.0 if view == "overview" else 18.0)
    camera.CreateProjectionAttr("perspective")
    camera.CreateHorizontalApertureAttr(20.955)
    if view == "top":
        camera.CreateProjectionAttr("orthographic")
        camera.CreateHorizontalApertureAttr(480.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200))
    existing_ops = camera.GetOrderedXformOps()
    op = existing_ops[0] if existing_ops else camera.AddTransformOp()
    eye, target, up = poses[view]
    op.Set(
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
        .GetInverse()
    )
    stage.SetEditTarget(previous)
    get_active_viewport().camera_path = str(camera.GetPath())

    def update(transform):
        if view != "follow":
            return
        px, py, _, qx, qy, qz, qw = transform
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        eye = Gf.Vec3d(px - 2.5 * math.cos(yaw), py - 2.5 * math.sin(yaw), 4.8)
        target = Gf.Vec3d(px, py, 0.6)
        original = stage.GetEditTarget()
        try:
            stage.SetEditTarget(stage.GetSessionLayer())
            op.Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
        finally:
            stage.SetEditTarget(original)

    return update
