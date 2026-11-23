"""Bound a model suggestion to a single, stationary simulator observation."""

import math

MAX_WALL_AGE_NS = 8_000_000_000
MAX_SIM_AGE_NS = 6_000_000_000


def verify_fresh(observation, episode, sim_ns, mono_ns, position, quaternion):
    if (observation["episode_id"] != episode or observation["epoch"] != 0
            or observation.get("diagnostic_camera_yaw_deg")):
        raise ValueError("model observation does not belong to the current motion episode")
    if not (0 <= mono_ns - int(observation["rgb_receive_mono_ns"]) <= MAX_WALL_AGE_NS
            and 0 <= sim_ns - int(observation["stamp_ns"]) <= MAX_SIM_AGE_NS):
        raise ValueError("model observation expired")
    captured = next((tf for tf in observation["transforms"]
                     if tf["target"] == "map" and tf["source"] == "base_link"), None)
    if captured is None:
        raise ValueError("model observation is missing the stamped base pose")
    original_quaternion = captured["quaternion_xyzw"]
    distance = math.dist(position, captured["xyz_m"])
    if (not math.isfinite(distance)
            or any(not math.isclose(math.hypot(*q), 1.0, abs_tol=1e-3)
                   for q in (quaternion, original_quaternion))):
        raise ValueError("invalid base pose in live intent")
    dot = sum(a * b for a, b in zip(quaternion, original_quaternion, strict=True))
    angle = 2 * math.acos(min(1.0, abs(dot)))
    if distance > 0.15 or angle > 0.15:
        raise ValueError("base pose moved beyond the model observation envelope")
