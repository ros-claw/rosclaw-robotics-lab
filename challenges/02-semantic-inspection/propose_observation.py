#!/usr/bin/env python3
"""USD-known semantic observation proposals; read-only, no motion authority."""

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def target_split(inventory):
    # Freeze deterministic split before tuning or evaluating candidate quality.
    targets = {
        row["path"]: row
        for row in inventory["scene_objects"]
        if row["type"] == "Xform" and "rackshelf" in Path(row["path"]).name.lower()
    }
    # A shelf's parent and child prims are one physical entity. Split only the
    # outermost matching root so the same shelf cannot leak into both groups.
    targets = {
        path: row
        for path, row in targets.items()
        if not any(
            path.startswith(parent + "/") for parent in targets if parent != path
        )
    }
    return {
        name: (
            "holdout"
            if int(hashlib.sha256(name.encode()).hexdigest()[:8], 16) % 4 == 0
            else "development"
        )
        for name in sorted(targets)
    }


def free_grid(map_yaml):
    config = yaml.safe_load(map_yaml.read_text())
    if config.get("mode", "trinary") != "trinary" or config.get("negate", 0) not in (
        0,
        1,
    ):
        raise ValueError("Only explicit trinary occupancy maps are supported")
    if len(config["origin"]) != 3 or abs(config["origin"][2]) > 1e-9:
        raise ValueError("Rotated map origins require a validated transform adapter")
    image = (map_yaml.parent / config["image"]).resolve()
    pixels = np.asarray(Image.open(image).convert("L"), dtype=float) / 255
    occupancy = pixels if config.get("negate", 0) else 1 - pixels
    # Unknown and occupied cells are both excluded. Flip image y into ROS map y.
    free = np.flipud(occupancy < float(config["free_thresh"]))
    resolution = float(config["resolution"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Invalid map resolution")
    return config, image, free, resolution


def propose(
    *,
    inventory_path,
    map_yaml,
    physics_path,
    body_path,
    nav_params,
    target,
    split="development",
    margin=0.05,
    max_snapshot_age=2.0,
    now=None,
):
    import time

    now = time.time() if now is None else now
    inventory = json.loads(inventory_path.read_text())
    if inventory.get("up_axis") != "Z" or inventory.get("meters_per_unit") != 1:
        raise ValueError("USD/map units and up-axis must be verified")
    split_map = target_split(inventory)
    if target not in split_map or split_map[target] != split:
        raise ValueError("Target is absent or belongs to the sealed alternate split")
    body = json.loads(body_path.read_text())
    if not body.get("body_id") or not body.get("effective_body_hash"):
        raise ValueError("Measured Body snapshot binding required")
    physics_bytes = physics_path.read_bytes()
    physics = json.loads(physics_bytes)
    if not 0 <= now - physics["wall_time"] <= max_snapshot_age or not physics.get(
        "timeline_playing"
    ):
        raise ValueError("Fresh, playing independent robot state required")
    if (
        not physics.get("collision_observer_complete")
        or physics.get("collision_count")
        or physics.get("contact_errors")
    ):
        raise ValueError("Complete collision observation without collision is required")
    if physics.get("physics_body_path") != "/World/Nova_Carter_ROS/chassis_link":
        raise ValueError("Unexpected physical Body")
    params = yaml.safe_load(nav_params.read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = json.loads(local["footprint"])
    padding = float(local["footprint_padding"])
    if (
        not footprint
        or not all(len(v) == 2 and all(math.isfinite(x) for x in v) for v in footprint)
        or margin < 0
        or padding < 0
    ):
        raise ValueError("Finite measured footprint and nonnegative margins required")
    radius = max(math.hypot(*vertex) for vertex in footprint) + padding + margin
    cfg, image, free, res = free_grid(map_yaml)
    # Treat outside-map cells as occupied and deduct a half-cell diagonal:
    # clearance is from cell boundary, not an optimistic point distance.
    distances = distance_transform_edt(np.pad(free, 1, constant_values=False))[
        1:-1, 1:-1
    ] * res - res / math.sqrt(2)
    safe = free & (distances >= radius)
    origin = cfg["origin"][:2]

    def cell(x, y):
        return math.floor((x - origin[0]) / res), math.floor((y - origin[1]) / res)

    x, y = physics["physics_transforms_xyzw"][0][:2]
    sx, sy = cell(x, y)
    h, w = safe.shape
    if not (0 <= sx < w and 0 <= sy < h and safe[sy, sx]):
        raise ValueError("Current Body is outside conservative safe map component")
    reachable = np.zeros(safe.shape, dtype=bool)
    reachable[sy, sx] = True
    queue = deque([(sx, sy)])
    while queue:
        cx, cy = queue.popleft()
        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if 0 <= nx < w and 0 <= ny < h and safe[ny, nx] and not reachable[ny, nx]:
                reachable[ny, nx] = True
                queue.append((nx, ny))
    row = next(item for item in inventory["scene_objects"] if item["path"] == target)
    lower, upper = row["min"], row["max"]
    if not all(math.isfinite(v) for v in lower + upper) or any(
        a >= b for a, b in zip(lower[:2], upper[:2])
    ):
        raise ValueError("Invalid target bounds")
    center = [(lower[i] + upper[i]) / 2 for i in (0, 1)]
    candidates = []
    # Sample around actual USD bounds, never registered patrol coordinates.
    # Broad-face observation with standoff greater than the Body enclosure.
    for axis in (0, 1):
        other = 1 - axis
        for sign in (-1, 1):
            edge = lower[axis] if sign < 0 else upper[axis]
            for fraction in (0.25, 0.5, 0.75):
                for offset in (radius + 0.35, radius + 0.7, radius + 1.1):
                    xy = [0.0, 0.0]
                    xy[axis] = edge + sign * offset
                    xy[other] = lower[other] + fraction * (upper[other] - lower[other])
                    cx, cy = cell(*xy)
                    if not (0 <= cx < w and 0 <= cy < h and reachable[cy, cx]):
                        continue
                    # Check static line-of-sight up to just outside target bounds.
                    end = list(xy)
                    end[axis] = edge + sign * res
                    steps = max(1, math.ceil(math.dist(xy, end) / res * 2))
                    samples = [
                        (
                            xy[0] + (end[0] - xy[0]) * t / steps,
                            xy[1] + (end[1] - xy[1]) * t / steps,
                        )
                        for t in range(steps + 1)
                    ]
                    if not all(
                        0 <= (c := cell(*p))[0] < w
                        and 0 <= c[1] < h
                        and free[c[1], c[0]]
                        for p in samples
                    ):
                        continue
                    yaw = math.atan2(center[1] - xy[1], center[0] - xy[0])
                    candidates.append(
                        {
                            "x": xy[0],
                            "y": xy[1],
                            "yaw": yaw,
                            "static_clearance_m": float(distances[cy, cx]),
                            "distance_from_current_pose_m": math.dist([x, y], xy),
                            "target_prim": target,
                        }
                    )
    candidates.sort(
        key=lambda c: (c["distance_from_current_pose_m"], -c["static_clearance_m"])
    )
    evidence = {
        "inventory_sha256": digest(inventory_path),
        "map_yaml_sha256": digest(map_yaml),
        "map_image_sha256": digest(image),
        "physics_snapshot_sha256": hashlib.sha256(physics_bytes).hexdigest(),
        "nav_params_sha256": digest(nav_params),
        "body_snapshot_hash": body["effective_body_hash"],
        "observer_id": physics["observer_id"],
    }
    for candidate in candidates:
        candidate["proposal_id"] = hashlib.sha256(
            json.dumps(
                {"candidate": candidate, "evidence": evidence}, sort_keys=True
            ).encode()
        ).hexdigest()
    return {
        "schema_version": "rosclaw.semantic_observation_proposals.v1",
        "status": "PROPOSAL_ONLY" if candidates else "NO_SAFE_CANDIDATE",
        "authorization": False,
        "execution_allowed": False,
        "evidence_domain": "OFFLINE_ANALYSIS_OF_SIMULATION",
        "semantic_source": "USD-known prim names/bounds; not visual or sensor discovery",
        "split": split,
        "target_prim": target,
        "target_bounds_xy": [lower[:2], upper[:2]],
        "body_id": body["body_id"],
        "body_enclosing_radius_with_margins_m": radius,
        "evidence": evidence,
        "captured_at_wall": physics["wall_time"],
        "generated_at_wall": now,
        "candidates": candidates[:8],
        "required_before_execution": [
            "Revalidate fresh map/costmap, robot pose, TF, clock and complete contacts inside daemon",
            "Validate proposal intent and measured Body snapshot via existing Body validator",
            "Obtain Nav2 ComputePathToPose path and validate swept footprint against current costmap",
            "Dispatch only through existing rosclawd guarded action with exact bound proposal hash",
            "Independent arrival/dwell/collision and target-facing observation verification; final task receipt",
        ],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("inventory", "map-yaml", "physics", "body", "nav-params", "output"):
        p.add_argument("--" + name, type=Path, required=True)
    p.add_argument("--target", required=True)
    p.add_argument("--split", choices=["development", "holdout"], default="development")
    a = p.parse_args()
    result = propose(
        inventory_path=a.inventory,
        map_yaml=a.map_yaml,
        physics_path=a.physics,
        body_path=a.body,
        nav_params=a.nav_params,
        target=a.target,
        split=a.split,
    )
    a.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(
        json.dumps(
            {
                "status": result["status"],
                "candidates": len(result["candidates"]),
                "authorization": False,
            }
        )
    )


if __name__ == "__main__":
    main()
