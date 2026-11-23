"""Author a fixed, single-render-product RGB-D sensor on the Nova Carter."""

import sys
from pathlib import Path

import omni.graph.core as og
from pxr import Gf, Sdf, Usd, UsdGeom
from usdrt import Sdf as RtSdf

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.camera_mount import HEIGHT, OPTICAL_FRAME, TOPICS, TRANSLATION_M, WIDTH, yaw_mount_xyzw


def add_front_rgbd(stage, calibration=False, yaw=0.0):
    chassis = stage.GetPrimAtPath("/World/Nova_Carter_ROS/chassis_link")
    path = chassis.GetPath().AppendChild("LunaFrontCamera")
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera = UsdGeom.Camera.Define(stage, path)
        xform = UsdGeom.Xformable(camera)
        xform.AddTranslateOp().Set(Gf.Vec3d(*TRANSLATION_M))
        # Chassis +X forward, +Y left, +Z up. USD cameras look along -Z.
        px, py, pz, pw = yaw_mount_xyzw((0.5, -0.5, -0.5, 0.5), yaw)
        xform.AddOrientOp().Set(Gf.Quatf(pw, Gf.Vec3f(px, py, pz)))
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(36.0 * HEIGHT / WIDTH)
        camera.GetFocalLengthAttr().Set(24.0)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 45.0))
        prim = camera.GetPrim()
        prim.AddAppliedSchema("OmniSensorAPI")
        prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(10.0)

        if calibration:
            from calibration_targets import add_targets

            add_targets(stage, path)

        keys = og.Controller.Keys
        nodes = [
            ("Tick", "omni.graph.action.OnTick"),
            ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        connections = [("Tick.outputs:tick", "Render.inputs:execIn")]
        values = [("Render.inputs:cameraPrim", [RtSdf.Path(str(path))]),
                  ("Render.inputs:width", WIDTH), ("Render.inputs:height", HEIGHT),
                  ("Rgb.inputs:type", "rgb"), ("Depth.inputs:type", "depth")]
        for node in ("Rgb", "Depth", "Info"):
            connections.extend([
                ("Render.outputs:execOut", f"{node}.inputs:execIn"),
                ("Render.outputs:renderProductPath", f"{node}.inputs:renderProductPath"),
            ])
            values.extend([(f"{node}.inputs:topicName", TOPICS[node.lower()]),
                           (f"{node}.inputs:frameId", OPTICAL_FRAME)])
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/World/LunaFrontRGBD", "evaluator_name": "push",
             "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
            {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
        )
        og.Controller.evaluate_sync(graph)
    print(f"Luna RGB-D rig configured at {path} ({WIDTH}x{HEIGHT})", flush=True)
