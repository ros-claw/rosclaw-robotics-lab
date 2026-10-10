"""Read the composed live Stage; never equate disabled mesh with no proxy."""

import math
from pxr import Usd, UsdGeom, UsdPhysics


def audit_stage(stage):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True
    )
    rows = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not (prim.IsA(UsdGeom.Boundable) or prim.IsA(UsdGeom.Xform)):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        lo, hi = list(box.GetMin()), list(box.GetMax())
        if not all(math.isfinite(v) for v in lo + hi):
            continue
        collision = prim.HasAPI(UsdPhysics.CollisionAPI)
        image = UsdGeom.Imageable(prim)
        ancestor = prim
        rigid = None
        while ancestor and ancestor.GetPath() != stage.GetPseudoRoot().GetPath():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid = str(ancestor.GetPath())
                break
            ancestor = ancestor.GetParent()
        rows.append(
            {
                "path": str(prim.GetPath()),
                "type": prim.GetTypeName(),
                "min": lo,
                "max": hi,
                "visible": image.ComputeVisibility() != UsdGeom.Tokens.invisible,
                "purpose": str(image.ComputePurpose()),
                "collision_api": collision,
                "collision_enabled": bool(
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                )
                if collision
                else False,
                "mesh_approximation": str(
                    UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                )
                if prim.HasAPI(UsdPhysics.MeshCollisionAPI)
                else None,
                "rigid_body_ancestor": rigid,
                "instance_proxy": prim.IsInstanceProxy(),
                "source_specs": [
                    {"layer": p.layer.identifier, "path": str(p.path)}
                    for p in prim.GetPrimStack()
                ],
            }
        )
    return {
        "schema": "rosclaw.composed_scene_audit.v1",
        "frame": "USD world",
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "root_layer": stage.GetRootLayer().identifier,
        "session_layer": stage.GetSessionLayer().identifier,
        "raw_fork_prims": [
            {
                "path": str(p.GetPath()),
                "type": p.GetTypeName(),
                "active": p.IsActive(),
                "defined": p.IsDefined(),
                "collision_api": p.HasAPI(UsdPhysics.CollisionAPI),
                "attributes": {
                    a.GetName(): str(a.Get())
                    for a in p.GetAttributes()
                    if a.HasAuthoredValue()
                },
            }
            for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
            if str(p.GetPath()).startswith("/World/warehouse_with_forklifts/Forklift")
        ],
        "rows": rows,
        "limits": "AABB overlap is not collision equivalence. Physical controls required before clearance claims.",
    }


def prepare_fork_control(stage, *, repair=False, control=False):
    """Session-only conservative envelopes. Explicitly an approximation, not asset fix."""
    import numpy as np
    from pxr import Gf, PhysxSchema

    previous = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    rows = audit_stage(stage)["rows"]
    forks = [
        r
        for r in rows
        if r["type"] == "Mesh"
        and "fork" in r["path"].rsplit("/", 1)[-1].lower()
        and "body" not in r["path"].lower()
    ]
    result = []
    for index, fork in enumerate(forks):
        prim = stage.GetPrimAtPath(fork["path"])
        mesh = UsdGeom.Mesh(prim)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        points = np.array(
            [list(matrix.Transform(Gf.Vec3d(*p))) for p in mesh.GetPointsAttr().Get()]
        )
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        offset = 0
        samples = []
        for count in counts:
            face = points[list(indices[offset : offset + count])]
            offset += count
            center = face.mean(axis=0)
            if center[2] < 0.25 and center[0] < fork["min"][0] + 0.5:
                samples.append(center.tolist())
        # Spread measured surface samples instead of 16 coincident face centroids.
        samples = list({tuple(round(v, 5) for v in p): p for p in samples}.values())
        samples = sorted(samples, key=lambda x: (x[0], x[1]))[
            :: max(1, len(samples) // 32)
        ][:32]
        if not samples:
            raise ValueError("No actual low fork tip geometry: " + fork["path"])
        item = {
            "source_fork": fork,
            "visual_surface_samples": samples,
            "repair_enabled": repair,
        }
        lo = np.array(fork["min"]) - np.array([0.05, 0.05, 0.02])
        hi = np.array(fork["max"]) + np.array([0.05, 0.05, 0.02])
        if repair:
            cube = UsdGeom.Cube.Define(stage, f"/World/ROSClawForkSafety/proxy_{index}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
            cube.AddScaleOp().Set(Gf.Vec3d(*(hi - lo)))
            cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            item["proxy"] = {
                "path": str(cube.GetPath()),
                "min": lo.tolist(),
                "max": hi.tolist(),
                "kind": "conservative full visible-fork AABB; not exact mesh equivalence",
            }
        if control:
            # Test sphere over a measured fork tip; independent contacts/PhysX pose decide result.
            tip = samples[0]
            sphere = UsdGeom.Sphere.Define(
                stage, f"/World/ROSClawForkControl/sphere_{index}"
            )
            sphere.CreateRadiusAttr(0.06)
            sphere.AddTranslateOp().Set(
                Gf.Vec3d(tip[0], tip[1], 1.5 if index == 0 else tip[2] + 0.03)
            )
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
            UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateMassAttr(0.1)
            PhysxSchema.PhysxContactReportAPI.Apply(
                sphere.GetPrim()
            ).CreateThresholdAttr().Set(0.0)
            item["sphere"] = str(sphere.GetPath())
        result.append(item)
    stage.SetEditTarget(previous)
    return result


async def measure_fork_control(stage, controls, output):
    import asyncio
    import carb
    import json
    import omni.kit.app
    import omni.usd
    from omni.physx import (
        get_physx_scene_query_interface,
        get_physx_simulation_interface,
    )
    from omni.physics import tensors
    from pxr import PhysicsSchemaTools

    contacts = []

    def report(headers, data):
        for h in headers:
            paths = [
                str(PhysicsSchemaTools.intToSdfPath(p))
                for p in [h.collider0, h.collider1]
            ]
            if any("ROSClawForkControl" in p for p in paths) and h.num_contact_data:
                contacts.append({"colliders": paths, "contacts": h.num_contact_data})

    subscription = get_physx_simulation_interface().subscribe_contact_report_events(
        report
    )
    app = omni.kit.app.get_app()
    for _ in range(240):
        await app.next_update_async()
    (output.parent / "composed-scene-audit-playing.json").write_text(
        json.dumps(audit_stage(stage), indent=2, allow_nan=False) + "\n"
    )
    query = get_physx_scene_query_interface()
    view = tensors.create_simulation_view(
        "numpy", stage_id=omni.usd.get_context().get_stage_id(), backend="physx"
    )
    view.set_subspace_roots("/")
    for c in controls:
        rays = []
        for xyz in c["visual_surface_samples"][1:]:
            hit = query.raycast_closest(
                carb.Float3(xyz[0], xyz[1], 1.4), carb.Float3(0, 0, -1), 1.6
            )
            rays.append(
                {
                    "xy": xyz[:2],
                    "visible_surface_z": xyz[2],
                    "hit": bool(hit["hit"]),
                    "collider": str(hit.get("collision", "")),
                    "position": list(hit.get("position", [])),
                    "distance": hit.get("distance"),
                }
            )
        c["vertical_physx_rays"] = rays
        overlaps = []
        for xyz in c["visual_surface_samples"]:
            hits = []

            def found(hit):
                hits.append(str(hit.collision))
                return True

            query.overlap_sphere(0.015, carb.Float3(*xyz), found, False)
            overlaps.append({"visual_surface_xyz": xyz, "colliders": hits})
        c["surface_overlap_queries"] = overlaps
        if "sphere" in c:
            rb = view.create_rigid_body_view(c["sphere"])
            c["sphere_final_physx_xyzw"] = rb.get_transforms().tolist()
            c["sphere_final_velocity"] = rb.get_velocities().tolist()
    output.write_text(
        json.dumps(
            {
                "schema": "rosclaw.fork_collision_control.v1",
                "controls": controls,
                "contacts": contacts,
            },
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
