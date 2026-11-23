"""A movable shutter for the reproducible dynamic-visibility trial."""

import json
import time

from pxr import Gf, Usd, UsdGeom, UsdPhysics


class VisualShutter:
    def __init__(self, stage, log):
        self.stage = stage
        self.log = log
        self.phase = None
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            path = "/World/LunaVisualShutter"
            body = UsdGeom.Xform.Define(stage, path)
            self.translation = body.AddTranslateOp()
            self.translation.Set(Gf.Vec3d(-0.74, 0, 0.76))
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            rigid_body.CreateKinematicEnabledAttr(True)
            cover = UsdGeom.Cube.Define(stage, path + "/Cover")
            cover.CreateSizeAttr(1.0)
            cover.AddScaleOp().Set(Gf.Vec3f(0.14, 2.01, 1.52))
            cover.CreateDisplayColorAttr([(0.30, 0.35, 0.39)])
            UsdPhysics.CollisionAPI.Apply(cover.GetPrim())

    def update(self, sim_time):
        lift = min(1.0, max(0.0, (sim_time - 7.0) / 1.8))
        phase = "closed" if lift == 0 else "open" if lift == 1 else "opening"
        if phase == self.phase and phase != "opening":
            return
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            self.translation.Set(Gf.Vec3d(-0.74, 0, 0.76 + 2.4 * lift))
        if phase != self.phase:
            self.phase = phase
            with self.log.open("a") as stream:
                stream.write(json.dumps({"source": "isaac-shutter-driver", "phase": phase,
                                         "sim_time_s": sim_time,
                                         "host_mono_ns": str(time.monotonic_ns())}) + "\n")
