"""Geometry-only camera test fixtures, authored in the calibration session layer."""

from pxr import Gf, UsdGeom


def add_targets(stage, camera_path):
    # Surfaces face the camera at optical (-0.2, -0.12, 1) and (0.32, 0.16, 1.6) m.
    for index, (x, y, depth, color) in enumerate((
        (-0.20, 0.12, 1.0, (0.0, 0.8, 0.8)),
        (0.32, -0.16, 1.6, (0.85, 0.1, 0.1)),
    )):
        plate = UsdGeom.Cube.Define(stage, camera_path.AppendChild(f"CalibrationPlate{index}"))
        plate.CreateSizeAttr(1.0)
        plate.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xform = UsdGeom.XformCommonAPI(plate)
        xform.SetTranslate(Gf.Vec3d(x, y, -depth - 0.01))
        xform.SetScale(Gf.Vec3f(0.16, 0.16, 0.02))
