"""Trim unused sensor render products on the NVIDIA baseline stage."""
import omni.usd
from pxr import Usd

stage = omni.usd.get_context().get_stage()
chassis = stage.GetPrimAtPath("/World/Nova_Carter_ROS/chassis_link")
if not chassis.IsValid():
    raise RuntimeError("The NVIDIA warehouse asset did not resolve Nova Carter's chassis")

disabled = []
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for name in ("front_hawk", "left_hawk", "back_hawk", "right_hawk"):
        camera = chassis.GetChild("sensors").GetChild(name)
        if camera.IsValid():
            variant = camera.GetVariantSet("ROS")
            if "Disabled" in variant.GetVariantNames():
                variant.SetVariantSelection("Disabled")
                disabled.append(str(camera.GetPath()))
    for prim in stage.Traverse():
        if prim.GetName().endswith("camera_render_product"):
            enabled = prim.GetAttribute("inputs:enabled")
            if enabled:
                enabled.Set(False)
                disabled.append(str(prim.GetPath()))
print(f"Luna baseline loaded. Disabled unused camera graphs: {disabled}", flush=True)
