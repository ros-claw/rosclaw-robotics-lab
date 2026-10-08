"""Simulator-independent calculations on independently captured Isaac evidence.

Nav2 success, stable physical arrival and fresh LiDAR are separate requirements.
Incomplete evidence is UNKNOWN and cannot be promoted by an Agent statement.
"""

import math


def pose_error(sample, site):
    p = sample["physics_transforms_xyzw"][0]
    x, y, z, w = p[3:7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    delta = math.atan2(math.sin(yaw - site["yaw"]), math.cos(yaw - site["yaw"]))
    return math.dist(p[:2], [site["x"], site["y"]]), abs(delta)


def verify_visit(
    samples, site, nav_result, sensor, *, body_path, tolerance=0.4, dwell=2.0
):
    unknown, failures = [], []
    def finite_values(values):
        return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)

    try:
        complete = finite_values([site["x"], site["y"], site["yaw"],
                                  nav_result["wall_time"]]) and all(
            finite_values([sample["wall_time"], sample["sim_time"],
                           *sample["physics_transforms_xyzw"][0],
                           *sample["linear_velocity_xyz"],
                           *sample["angular_velocity_xyz"]])
            and len(sample["physics_transforms_xyzw"][0]) == 7
            and len(sample["linear_velocity_xyz"]) == 3
            and len(sample["angular_velocity_xyz"]) == 3
            for sample in samples
        )
        if sensor:
            complete = complete and finite_values([sensor["wall_time"], sensor["stamp"]])
    except (KeyError, IndexError, TypeError):
        complete = False
    if not complete:
        return {"verification_status": "UNKNOWN", "success": False,
                "position_error_m": None, "yaw_error_rad": None,
                "stable_dwell_sim_seconds": 0, "collision_count": None,
                "sensor": sensor, "failures": [],
                "missing_evidence": ["missing or non-finite physical evidence"],
                "physics_sample_count": len(samples)}
    if len({s.get("observer_id") for s in samples}) > 1:
        failures.append("independent observer changed during action")
    if nav_result.get("status") != 4 or nav_result.get("error_code") != 0:
        failures.append("Nav2 did not SUCCEED")
    if len(samples) < 2:
        unknown.append("independent physics samples missing")
    if any(s.get("physics_body_path") != body_path for s in samples):
        failures.append("physics Body identity mismatch")
    if any(
        not s.get("collision_observer_complete") or s.get("contact_errors")
        for s in samples
    ):
        unknown.append("collision observation incomplete")
    if any(s.get("collision_count", 0) for s in samples):
        failures.append("non-floor physical collision observed")
    if any(s.get("timeline_playing") is not True for s in samples):
        failures.append("simulation paused during action")
    if any(
        b["wall_time"] <= a["wall_time"]
        or b["wall_time"] - a["wall_time"] > 2
        or b["sim_time"] < a["sim_time"]
        for a, b in zip(samples, samples[1:])
    ):
        unknown.append("independent trajectory gap or time discontinuity")
    settled = []
    for sample in samples:
        if sample["wall_time"] < nav_result.get("wall_time", math.inf):
            continue
        error, yaw_error = pose_error(sample, site)
        speed = math.hypot(*sample["linear_velocity_xyz"][:2])
        angular_speed = abs(sample["angular_velocity_xyz"][2])
        if (
            error <= tolerance
            and yaw_error <= 0.35
            and speed <= 0.02
            and angular_speed <= 0.1
        ):
            settled.append(sample)
        else:
            settled = []
    span = settled[-1]["sim_time"] - settled[0]["sim_time"] if len(settled) >= 2 else 0
    if span < dwell:
        failures.append("stable arrival dwell not observed")
    latest = samples[-1] if samples else {}
    if (
        not sensor
        or not sensor.get("valid")
        or not 0
        <= latest.get("wall_time", -math.inf) - sensor.get("wall_time", math.inf)
        <= 3
        or abs(latest.get("sim_time", math.inf) - sensor.get("stamp", -math.inf)) > 1
    ):
        unknown.append("fresh valid LiDAR evidence missing")
    status = "FAIL" if failures else "UNKNOWN" if unknown else "PASS"
    position_error, orientation_error = (
        pose_error(latest, site) if latest else (None, None)
    )
    return {
        "verification_status": status,
        "success": status == "PASS",
        "position_error_m": position_error,
        "yaw_error_rad": orientation_error,
        "stable_dwell_sim_seconds": span,
        "collision_count": max(
            (s.get("collision_count", 0) for s in samples), default=None
        ),
        "sensor": sensor,
        "failures": failures,
        "missing_evidence": unknown,
        "physics_sample_count": len(samples),
    }
