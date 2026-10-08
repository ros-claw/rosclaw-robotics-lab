"""Session-layer instrumentation; physical observations never rely on ROS odometry."""

import json
import os
import sys
import time
import uuid
from pathlib import Path

import carb
import omni.kit.app
import omni.kit.async_engine
import omni.timeline
import omni.usd
from omni.physx import get_physx_simulation_interface
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics

OBSERVER_ID = uuid.uuid4().hex
OUTPUT = Path(os.environ["ROSCLAW_LAB_REPORT_DIR"])
OUTPUT.mkdir(parents=True, exist_ok=True)
PHYSICS_ENGINE = SimulationManager.get_active_physics_engine()
if PHYSICS_ENGINE != "physx":
    raise RuntimeError(
        "Expected PhysX backend for the validated contact/tensor observer: "
        + PHYSICS_ENGINE
    )
TIMELINE = omni.timeline.get_timeline_interface()
TIMELINE.set_looping(False)
TIMELINE.set_end_time(36000.0)
STAGE = omni.usd.get_context().get_stage()
_previous_target = STAGE.GetEditTarget()
STAGE.SetEditTarget(STAGE.GetSessionLayer())
if os.environ.get("ROSCLAW_CONTACT_TEST") == "1":
    cube = UsdGeom.Cube.Define(STAGE, "/World/ROSClawContactTestCube")
    cube.CreateSizeAttr(0.2)
    cube.AddTranslateOp().Set(Gf.Vec3d(-5.75, -1, 1.0))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)

TEST_OBSTACLE = None
if os.environ.get("ROSCLAW_OBSTACLE_TEST") == "1":
    box = UsdGeom.Cube.Define(STAGE, "/World/ROSClawUnmappedBox")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(-3.5, 4, 0.5))
    box.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.28, 0.08)])
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    TEST_OBSTACLE = {
        "path": str(box.GetPath()),
        "center_xyz": [-3.5, 4, 0.5],
        "size_m": 1.0,
        "physical_collision": True,
        "source": "session layer; absent from official occupancy map",
    }

DISABLED_ROS_VARIANTS = []
for camera_name in ("front_hawk", "left_hawk", "back_hawk", "right_hawk"):
    path = "/World/Nova_Carter_ROS/chassis_link/sensors/" + camera_name
    camera = STAGE.GetPrimAtPath(path)
    if camera:
        variant = camera.GetVariantSet("ROS")
        if "Disabled" not in variant.GetVariantNames():
            raise RuntimeError(
                "Official Hawk ROS Disabled variant unavailable: " + path
            )
        variant.SetVariantSelection("Disabled")
        DISABLED_ROS_VARIANTS.append(path)
DISABLED_CAMERAS = []
CONTACT_BODIES = []
for _prim in STAGE.Traverse():
    if _prim.GetName().endswith("camera_render_product"):
        _enabled = _prim.GetAttribute("inputs:enabled")
        if _enabled:
            _enabled.Set(False)
            DISABLED_CAMERAS.append(str(_prim.GetPath()))
    if str(_prim.GetPath()).startswith("/World/Nova_Carter_ROS/") and _prim.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):
        api = PhysxSchema.PhysxContactReportAPI.Apply(_prim)
        api.CreateThresholdAttr().Set(0.0)
        CONTACT_BODIES.append(str(_prim.GetPath()))
STAGE.SetEditTarget(_previous_target)
sys.path.insert(0, str(Path(os.environ["ROSCLAW_LAB_CHALLENGE_DIR"]) / "isaac"))
from stage_inventory import inspect_scene
from camera_setup import setup_camera

current_display_view = os.environ.get("ROSCLAW_CAMERA_VIEW", "official")
update_display_camera = setup_camera(
    STAGE, os.environ.get("ROSCLAW_CAMERA_VIEW", "official")
)

INVENTORY = inspect_scene(STAGE)
(OUTPUT / "stage-inventory.json").write_text(json.dumps(INVENTORY, indent=2) + "\n")
FLOOR_PATHS = {
    path
    for path, info in INVENTORY["floors"].items()
    if info["verified_horizontal_floor"]
}
CONTACT_PAIRS = {}
NON_FLOOR_PAIRS = set()
FLOOR_EVENTS = 0
CONTACT_ERRORS = []
CONTACT_CALLBACKS = 0


def write_atomic(name, data):
    tmp = OUTPUT / (name + ".tmp")
    tmp.write_text(json.dumps(data, allow_nan=False) + "\n")
    tmp.replace(OUTPUT / name)


def on_contact(headers, contacts):
    global CONTACT_CALLBACKS, FLOOR_EVENTS
    CONTACT_CALLBACKS += 1
    try:
        for header in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            key = "|".join(sorted((a, b)))
            collider_a = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
            collider_b = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            floor_contact = collider_a in FLOOR_PATHS or collider_b in FLOOR_PATHS
            if header.num_contact_data:
                if floor_contact:
                    FLOOR_EVENTS += 1
                else:
                    NON_FLOOR_PAIRS.add("|".join(sorted((collider_a, collider_b))))
            event_type = str(header.type)
            first = key not in CONTACT_PAIRS
            CONTACT_PAIRS[key] = CONTACT_PAIRS.get(key, 0) + 1
            if first or "FOUND" in event_type or "LOST" in event_type:
                row = {
                    "wall_time": time.time(),
                    "sim_time": TIMELINE.get_current_time(),
                    "actor0": a,
                    "actor1": b,
                    "type": event_type,
                    "collider0": str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                    "collider1": str(PhysicsSchemaTools.intToSdfPath(header.collider1)),
                    "num_contacts": header.num_contact_data,
                    "classification": "FLOOR" if floor_contact else "NON_FLOOR",
                }
                if header.num_contact_data:
                    point = contacts[header.contact_data_offset]
                    row["normal"] = list(point.normal)
                    row["position"] = list(point.position)
                with (OUTPUT / "physics-contacts.jsonl").open("a") as stream:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
    except Exception as exc:
        CONTACT_ERRORS.append(str(exc))


CONTACT_SUB = get_physx_simulation_interface().subscribe_contact_report_events(
    on_contact
)


async def capture_frames():
    global update_display_camera, current_display_view
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    viewport = get_active_viewport()
    resolution = tuple(int(value) for value in os.environ.get("ROSCLAW_CAMERA_RESOLUTION", "1920x1080").lower().split("x"))
    if len(resolution) != 2 or not all(240 <= value <= 3840 for value in resolution):
        raise ValueError("ROSCLAW_CAMERA_RESOLUTION must be WIDTHxHEIGHT, each between 240 and 3840")
    # Explicit underlying render resolution, not a resized screenshot or UI size.
    viewport.fill_frame = False
    viewport.resolution = resolution
    await viewport.wait_for_rendered_frames(3)
    directory = OUTPUT / "baseline-frames"
    directory.mkdir(exist_ok=True)
    deadline = time.monotonic() + float(
        os.environ.get("ROSCLAW_CAPTURE_SECONDS", "180")
    )
    index = 0
    while time.monotonic() < deadline:
        if not TIMELINE.is_playing():
            await omni.kit.app.get_app().next_update_async()
            continue
        path = directory / f"frame-{index:06d}.png"
        viewport = get_active_viewport()
        if current_display_view != "official":
            # The official stage launcher may restore its saved viewport camera
            # after our startup hook. Rebind display-only camera before capture.
            if str(viewport.camera_path) != "/World/ROSClawDisplayCamera":
                viewport.camera_path = "/World/ROSClawDisplayCamera"
        capture = capture_viewport_to_file(viewport, str(path))
        await capture.wait_for_result(completion_frames=0)
        write_deadline = time.monotonic() + 30
        while not path.is_file() and time.monotonic() < write_deadline:
            await omni.kit.app.get_app().next_update_async()
        if not path.is_file():
            raise RuntimeError("viewport PNG was not written within 30 seconds")
        with (directory / "timestamps.jsonl").open("a") as stream:
            stream.write(
                json.dumps(
                    {
                        "frame": path.name,
                        "camera_path": str(viewport.camera_path),
                        "display_view": current_display_view,
                        "render_resolution": list(viewport.resolution),
                        "wall_time": time.time(),
                        "sim_time": TIMELINE.get_current_time(),
                    }
                )
                + "\n"
            )
        if index == 0:
            (OUTPUT / "warehouse-baseline.png").write_bytes(path.read_bytes())
        index += 1
        next_frame = time.monotonic() + 0.5
        while time.monotonic() < next_frame:
            await omni.kit.app.get_app().next_update_async()


async def observe():
    from omni.physics import tensors

    app = omni.kit.app.get_app()
    deadline = time.monotonic() + 120
    while not TIMELINE.is_playing() and time.monotonic() < deadline:
        await app.next_update_async()
    if not TIMELINE.is_playing():
        raise RuntimeError("timeline did not start")
    start_sim = TIMELINE.get_current_time()
    for _ in range(120):
        await app.next_update_async()
    robots = [
        str(p.GetPath()) for p in STAGE.Traverse() if p.GetName() == "Nova_Carter_ROS"
    ]
    bodies = [
        str(p.GetPath()) for p in STAGE.Traverse() if p.GetName() == "chassis_link"
    ]
    if len(robots) != 1 or len(bodies) != 1:
        raise RuntimeError("robot/chassis identity is ambiguous")
    view = tensors.create_simulation_view(
        "numpy", stage_id=omni.usd.get_context().get_stage_id(), backend="physx"
    )
    view.set_subspace_roots("/")
    rb = view.create_rigid_body_view(bodies[0])
    transforms = rb.get_transforms().tolist()
    if len(transforms) != 1:
        raise RuntimeError("expected exactly one physical chassis")
    from isaacsim.storage.native import get_assets_root_path

    audit = {
        "official_resolved_asset_root": get_assets_root_path(skip_check=True),
        "composition_errors": [str(error) for error in STAGE.GetCompositionErrors()],
        "unloaded_payloads": [
            str(p.GetPath())
            for p in STAGE.Traverse()
            if p.HasPayload() and not p.IsLoaded()
        ],
        "physics_engine": PHYSICS_ENGINE,
        "captured_wall_time": time.time(),
        "stage": STAGE.GetRootLayer().identifier,
        "robots": robots,
        "physics_body_path": bodies[0],
        "disabled_camera_products": DISABLED_CAMERAS,
        "disabled_hawk_ros_variants": DISABLED_ROS_VARIANTS,
        "contact_reporting_bodies": CONTACT_BODIES,
        "timeline_playing": TIMELINE.is_playing(),
        "start_sim_time": start_sim,
        "end_sim_time": TIMELINE.get_current_time(),
        "usd_layers": [layer.identifier for layer in STAGE.GetUsedLayers()],
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(STAGE),
        "physics_transforms_xyzw": transforms,
        "independent_physics_status": "OBSERVED",
        "collision_acceptance": "Per-sample validated floor classification; see collision_observer_complete and collision_count",
        "verified_floor_paths": sorted(FLOOR_PATHS),
        "display_camera_view": os.environ.get("ROSCLAW_CAMERA_VIEW", "official"),
        "unmapped_test_obstacle": TEST_OBSTACLE,
    }
    write_atomic("scene-audit.json", audit)
    carb.log_warn("ROSCLAW_LAB_BASELINE independent physics observed")
    capture_task = omni.kit.async_engine.run_coroutine(capture_frames())
    capture_task.add_done_callback(on_task_done)
    index = 0
    while True:
        sample = {
            "sequence": index,
            "observer_id": OBSERVER_ID,
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "sim_time": TIMELINE.get_current_time(),
            "timeline_playing": TIMELINE.is_playing(),
            "physics_body_path": bodies[0],
            "physics_transforms_xyzw": rb.get_transforms().tolist(),
            "contact_callbacks": CONTACT_CALLBACKS,
            "contact_pairs": dict(CONTACT_PAIRS),
            "contact_errors": CONTACT_ERRORS[-10:],
            "floor_contact_events": FLOOR_EVENTS,
            "non_floor_contact_pairs": sorted(NON_FLOOR_PAIRS),
            "collision_count": len(NON_FLOOR_PAIRS),
            "collision_observer_complete": bool(FLOOR_PATHS)
            and FLOOR_EVENTS > 0
            and not CONTACT_ERRORS,
            "linear_velocity_xyz": rb.get_velocities().tolist()[0][:3],
            "angular_velocity_xyz": rb.get_velocities().tolist()[0][3:],
        }
        update_display_camera(sample["physics_transforms_xyzw"][0])
        with (OUTPUT / "physics-trajectory.jsonl").open("a") as stream:
            stream.write(json.dumps(sample, allow_nan=False) + "\n")
        write_atomic("physics-latest.json", sample)
        index += 1
        next_sample = time.monotonic() + 0.2
        while time.monotonic() < next_sample:
            await app.next_update_async()


def on_task_done(task):
    if task.cancelled():
        return
    error = task.exception()
    if error is not None:
        write_atomic("observer-error.json", {"status": "UNKNOWN", "error": repr(error)})
        carb.log_error("ROSCLAW_LAB_BASELINE observer failed: " + repr(error))


OBSERVATION_TASK = omni.kit.async_engine.run_coroutine(observe())
OBSERVATION_TASK.add_done_callback(on_task_done)
