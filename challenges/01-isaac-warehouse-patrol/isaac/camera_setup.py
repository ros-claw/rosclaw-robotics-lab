"""Display-only cameras: no collisions, sensor replacement or physics changes."""

import math
import os
from pxr import Gf, UsdGeom


def setup_camera(stage, view, path="/World/ROSClawDisplayCamera", activate=True):
    from omni.kit.viewport.utility import get_active_viewport

    poses = {
        "overview": ((6, -8, 4.8), (0, 4, 0), (0, 0, 1)),
        "top": ((0, 1, 5.5), (0, 1, 0), (1, 0, 0)),
        "follow": ((-2, -1, 3), (-6, -1, 0.6), (0, 0, 1)),
        "top-close": ((-6, -1, 5.5), (-6, -1, 0), (1, 0, 0)),
        "robot": ((-5.4, -1, 1.1), (0, -1, 1.0), (0, 0, 1)),
    }
    if view not in poses:
        if view == "official":
            return lambda transform: None
        raise ValueError(f"Unknown display camera view: {view}")
    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    camera = UsdGeom.Camera.Define(stage, path)
    camera.CreateFocalLengthAttr(12.0 if view in ("overview", "robot") else 18.0)
    camera.CreateProjectionAttr("perspective")
    camera.CreateHorizontalApertureAttr(20.955)
    if view in ("top", "top-close"):
        camera.CreateProjectionAttr("orthographic")
        # USD aperture is in tenths of a scene unit: 480 => 48 metres.
        width = float(os.environ.get("ROSCLAW_TOP_WIDTH_M", "12" if view == "top-close" else "48"))
        if not math.isfinite(width) or not 3 <= width <= 80:
            raise ValueError("ROSCLAW_TOP_WIDTH_M must be between 3 and 80 metres")
        camera.CreateHorizontalApertureAttr(width * 10)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200))
    existing_ops = camera.GetOrderedXformOps()
    op = existing_ops[0] if existing_ops else camera.AddTransformOp()
    eye, target, up = poses[view]
    op.Set(
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
        .GetInverse()
    )
    stage.SetEditTarget(previous)
    if activate:
        get_active_viewport().camera_path = str(camera.GetPath())

    def update(transform):
        if view not in ("follow", "top-close", "robot"):
            return
        px, py, pz, qx, qy, qz, qw = transform
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        up = Gf.Vec3d(0, 0, 1)
        if view == "top-close":
            eye, target = Gf.Vec3d(px, py, 5.5), Gf.Vec3d(px, py, 0)
            up = Gf.Vec3d(1, 0, 0)
        elif view == "robot":
            # Forward display camera follows the actual physical chassis pose.
            # It is not a replacement for the factory Hawk ROS sensor stream.
            rotation = Gf.Rotation(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
            position = Gf.Vec3d(px, py, pz)
            eye = position + rotation.TransformDir(Gf.Vec3d(0.65, 0, 1.0))
            target = eye + rotation.TransformDir(Gf.Vec3d(5, 0, -0.12))
            up = rotation.TransformDir(Gf.Vec3d(0, 0, 1))
        else:
            eye = Gf.Vec3d(px - 3.0 * math.cos(yaw), py - 3.0 * math.sin(yaw), 1.4)
            target = Gf.Vec3d(px, py, 0.7)
        original = stage.GetEditTarget()
        try:
            stage.SetEditTarget(stage.GetSessionLayer())
            op.Set(Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse())
        finally:
            stage.SetEditTarget(original)

    return update
