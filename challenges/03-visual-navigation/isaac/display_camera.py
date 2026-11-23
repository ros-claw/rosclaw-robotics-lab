"""Add an independent fixed overview camera for real-time recording."""

import omni.graph.core as og
from pxr import Gf, Sdf, Usd, UsdGeom
from usdrt import Sdf as RtSdf


def add_display_camera(stage):
    path = "/World/LunaOverview"
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera = UsdGeom.Camera.Define(stage, path)
        eye = Gf.Vec3d(-9, -1, 12)
        target = Gf.Vec3d(0, 0, 0)
        camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
            eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(20.25)
        camera.GetFocalLengthAttr().Set(21.0)
        prim = camera.GetPrim()
        prim.AddAppliedSchema("OmniSensorAPI")
        prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(12.0)
        keys = og.Controller.Keys
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/World/LunaOverviewRGB", "evaluator_name": "push",
             "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
            {keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnTick"),
                ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ], keys.CONNECT: [
                ("Tick.outputs:tick", "Render.inputs:execIn"),
                ("Render.outputs:execOut", "Rgb.inputs:execIn"),
                ("Render.outputs:renderProductPath", "Rgb.inputs:renderProductPath"),
            ], keys.SET_VALUES: [
                ("Render.inputs:cameraPrim", [RtSdf.Path(path)]),
                ("Render.inputs:width", 896), ("Render.inputs:height", 504),
                ("Rgb.inputs:type", "rgb"), ("Rgb.inputs:topicName", "/luna/display/image_raw"),
                ("Rgb.inputs:frameId", "luna_overview_optical"),
            ]},
        )
        og.Controller.evaluate_sync(graph)
