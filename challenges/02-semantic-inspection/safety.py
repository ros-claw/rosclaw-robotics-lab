"""Pure evidence gates for generated SIM targets; no execution authority."""

import hashlib
import json
import math

import numpy as np


def canonical_hash(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def swept_footprint(path, grid, footprint, padding=0.08):
    """Conservative cell/polygon overlap at <= half-cell translation / 0.05rad.

    Costmap 99/100 (inscribed/lethal) and unknown/outside are blocked. Lower
    inflated costs remain costs, otherwise inflation would be counted twice.
    Every cell intersecting polygon plus margin and half-cell diagonal is tested.
    """
    res = grid["resolution"]
    w = grid["width"]
    h = grid["height"]
    origin = grid["origin"]
    if not math.isfinite(res) or res <= 0 or w <= 0 or h <= 0 or padding < 0:
        raise ValueError("Invalid costmap or margin")
    costs = np.asarray(grid["data"]).reshape(h, w)
    vertices = np.asarray(footprint, dtype=float)
    if (
        vertices.shape[1:] != (2,)
        or len(vertices) < 3
        or not np.isfinite(vertices).all()
    ):
        raise ValueError("Invalid Body footprint")
    if not path or not all(
        len(p) == 3 and all(math.isfinite(x) for x in p) for p in path
    ):
        raise ValueError("Finite nonempty map path required")
    sampled = [path[0]]
    for a, b in zip(path, path[1:]):
        dyaw = angle_delta(b[2], a[2])
        n = max(
            1,
            math.ceil(math.dist(a[:2], b[:2]) / (res / 2)),
            math.ceil(abs(dyaw) / 0.05),
        )
        for i in range(1, n + 1):
            t = i / n
            sampled.append(
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw]
            )
    checked = 0
    max_cost = 0
    buffer = padding + res / math.sqrt(2)
    for x, y, yaw in sampled:
        c, s = math.cos(yaw), math.sin(yaw)
        poly = vertices @ np.array([[c, s], [-s, c]]) + np.array([x, y])
        low = np.floor((poly.min(axis=0) - buffer - origin) / res).astype(int)
        high = np.floor((poly.max(axis=0) + buffer - origin) / res).astype(int)
        if (low < 0).any() or high[0] >= w or high[1] >= h:
            raise ValueError("Swept footprint outside costmap")
        xs, ys = np.meshgrid(
            np.arange(low[0], high[0] + 1), np.arange(low[1], high[1] + 1)
        )
        pts = np.stack(
            [origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res], axis=-1
        )
        inside = np.zeros(xs.shape, dtype=bool)
        near = np.zeros(xs.shape, dtype=bool)
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            cross = ((a[1] > pts[..., 1]) != (b[1] > pts[..., 1])) & (
                pts[..., 0]
                < (b[0] - a[0]) * (pts[..., 1] - a[1]) / (b[1] - a[1] + 1e-30) + a[0]
            )
            inside ^= cross
            d = b - a
            t = np.clip(((pts - a) * d).sum(axis=-1) / float(d @ d), 0, 1)
            near |= np.linalg.norm(pts - a - t[..., None] * d, axis=-1) <= buffer
        selected = costs[ys[inside | near], xs[inside | near]]
        if not len(selected) or np.any((selected < 0) | (selected >= 99)):
            raise ValueError(
                "Swept footprint intersects unknown/inscribed/lethal cells"
            )
        max_cost = max(max_cost, int(selected.max()))
        checked += len(selected)
    return {
        "status": "PASS",
        "sampled_poses": len(sampled),
        "checked_cells_with_repetition": checked,
        "maximum_nonlethal_cost": max_cost,
        "translation_step_max_m": res / 2,
        "rotation_step_max_rad": 0.05,
        "footprint_padding_and_margin_m": padding,
        "unknown_and_outside_blocked": True,
    }


def lidar_target_hits(scan, bounds, *, sim_time, minimum_hits=3):
    """Actual planar LaserScan endpoints inside a USD-known XY region.

    Evidence of returns at the known region, not visual recognition, object
    identity, defect detection or semantic discovery from LiDAR alone.
    """
    if (
        not -0.2 <= sim_time - scan["stamp"] <= 1.5
        or abs(scan["stamp"] - scan["tf_stamp"]) > 0.5
    ):
        raise ValueError("LiDAR/TF observation is not synchronized and fresh")
    x, y, yaw = scan["map_sensor_xy_yaw"]
    hits = []
    for i, r in enumerate(scan["ranges"]):
        if (
            r is None
            or not math.isfinite(r)
            or not scan["range_min"] <= r <= scan["range_max"]
        ):
            continue
        a = yaw + scan["angle_min"] + i * scan["angle_increment"]
        px = x + r * math.cos(a)
        py = y + r * math.sin(a)
        if (
            bounds[0][0] - 0.10 <= px <= bounds[1][0] + 0.10
            and bounds[0][1] - 0.10 <= py <= bounds[1][1] + 0.10
        ):
            hits.append({"beam": i, "range_m": r, "map_xy": [px, py]})
    if len(hits) < minimum_hits:
        raise ValueError(
            f"Insufficient actual LiDAR returns in known target region: {len(hits)}"
        )
    return {
        "status": "PASS",
        "hit_count": len(hits),
        "minimum_hits": minimum_hits,
        "hits": hits,
        "semantic_source": "USD-known bounds; LaserScan measured returns, not sensor semantic discovery",
        "inspection_scope": "known-region LiDAR observation only; no visual/defect inspection",
    }
