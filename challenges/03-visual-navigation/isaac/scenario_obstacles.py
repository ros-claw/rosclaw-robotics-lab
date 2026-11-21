"""Script reproducible collision geometry for integration trials."""

import json
import math
import time

from pxr import Gf, Usd, UsdGeom, UsdPhysics


class ScenarioObstacle:
    def __init__(self, stage, path, item):
        self.keyframes = item["keyframes"]
        times = [frame[0] for frame in self.keyframes]
        if (not times or times[0] != 0 or times != sorted(set(times))
                or any(len(frame) != 4 or not all(map(math.isfinite, frame)) for frame in self.keyframes)
                or len(item["size"]) != 3
                or not all(math.isfinite(v) and 0 < v <= 10 for v in item["size"])):
            raise ValueError("invalid scenario obstacle size or keyframes")
        self.next_frame = 1
        self.translation = UsdGeom.Xform.Define(stage, path).AddTranslateOp()
        self.translation.Set(Gf.Vec3d(*self.keyframes[0][1:]))
        body = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(path))
        body.CreateKinematicEnabledAttr(True)
        geom = UsdGeom.Cube.Define(stage, path + "/Body")
        geom.CreateSizeAttr(1.0)
        geom.CreateDisplayColorAttr([(0.65, 0.23, 0.08)])
        geom.AddScaleOp().Set(Gf.Vec3f(*item["size"]))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())

    def step(self, now):
        previous = self.next_frame
        while self.next_frame < len(self.keyframes) and self.keyframes[self.next_frame][0] <= now:
            self.next_frame += 1
        start = self.keyframes[self.next_frame - 1]
        if self.next_frame == len(self.keyframes):
            self.translation.Set(Gf.Vec3d(*start[1:]))
        else:
            target = self.keyframes[self.next_frame]
            fraction = (now - start[0]) / (target[0] - start[0])
            self.translation.Set(Gf.Vec3d(*(a + fraction * (b - a)
                                            for a, b in zip(start[1:], target[1:]))))
        return self.next_frame != previous


class ScenarioObstacles:
    def __init__(self, stage, recipe, log):
        self.stage = stage
        self.log = log
        self.items = []
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for index, item in enumerate(json.loads(recipe.read_text())["obstacles"]):
                path = f"/World/LunaScenarioObstacles/Obstacle_{index}"
                self.items.append(ScenarioObstacle(stage, path, item))

    def update(self, now):
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for index, item in enumerate(self.items):
                if item.step(now):
                    with self.log.open("a") as stream:
                        stream.write(json.dumps({"source": "scenario_obstacle_driver", "actor": index,
                                                 "sim_time_s": now, "segment": item.next_frame - 1,
                                                 "host_mono_ns": str(time.monotonic_ns())}) + "\n")
