"""Measured ground cells and height-band obstacle cells, distinct from scene truth."""

import base64
import hashlib
import math
import numpy as np
from rosclaw.connectors.ros.verification.inspection import classify_passage
from loading_visibility import observed_height_bands

DEFAULT_THRESHOLDS = {
    "resolution_m": 0.2,
    "ground_abs_z_m": 0.08,
    "obstacle_min_z_m": 0.10,
    "obstacle_max_z_m": 0.8,
    "minimum_ground_points_per_cell": 2,
    "minimum_obstacle_points_per_cell": 3,
    "minimum_coverage": 0.70,
    "clearance_radius_m": 0.75,
    "max_observations": 2,
    "minimum_gain_cells": 3,
}


def _inspect_grid(
    snapshot, region, *, thresholds=None, previous=None, known_facilities=()
):
    t = thresholds or DEFAULT_THRESHOLDS
    res = t["resolution_m"]
    lo = region["min"]
    hi = region["max"]
    width, height = [int(math.ceil((hi[i] - lo[i]) / res)) for i in (0, 1)]
    if min(width, height) < 1:
        raise ValueError("Empty measured inspection grid")
    free_counts = np.zeros((height, width), dtype=np.int64)
    occ_counts = np.zeros_like(free_counts)
    known_counts = np.zeros_like(free_counts)
    visibility = np.zeros((height, width), dtype=np.uint8)
    valid = snapshot.get("status") == "PASS" and bool(snapshot.get("frames"))
    for f in snapshot.get("frames", []):
        if "raw_data_base64" not in f:
            raise ValueError("Raw PointCloud2 evidence required")
        if "raw_data_base64" in f:
            try:
                raw = base64.b64decode(f["raw_data_base64"], validate=True)
                if hashlib.sha256(raw).hexdigest() != f["raw_data_sha256"]:
                    raise ValueError("PointCloud2 raw payload hash mismatch")
                fields = {x["name"]: x for x in f["fields"]}
                formats = []
                for axis in ("x", "y", "z"):
                    field = fields[axis]
                    if field["count"] != 1 or field["datatype"] not in (7, 8):
                        raise ValueError("Unsupported XYZ field")
                    formats.append(
                        (">" if f["is_bigendian"] else "<")
                        + ("f4" if field["datatype"] == 7 else "f8")
                    )
                dtype = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": formats,
                        "offsets": [fields[k]["offset"] for k in ("x", "y", "z")],
                        "itemsize": f["point_step"],
                    }
                )
                decoded = np.ndarray(
                    (f["height"], f["width"]),
                    dtype=dtype,
                    buffer=raw,
                    strides=(f["row_step"], f["point_step"]),
                )
                local = np.stack(
                    [decoded[k].ravel() for k in ("x", "y", "z")], axis=1
                ).astype(float)
                finite = np.isfinite(local).all(axis=1)
                world = local[finite] @ np.asarray(
                    f["map_sensor_rotation"]
                ).T + np.asarray(f["map_sensor_origin"])
                if not np.allclose(world, f["points_map"], atol=1e-9, rtol=0):
                    raise ValueError("Raw cloud/TF transformed points mismatch")
            except Exception as exc:
                raise ValueError("PointCloud2 independent raw replay failed") from exc
        if (
            not -0.05 <= f["clock"] - f["stamp"] <= 1.5
            or abs(f["tf_stamp"] - f["stamp"]) > 0.001
        ):
            valid = False
            continue
        xyz = np.asarray(f["points_map"], dtype=float)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
            valid = False
            continue
        if (
            len(xyz) != f["finite_points"]
            or f["decoded_points"] != f["width"] * f["height"]
        ):
            valid = False
            continue
        cell = np.floor((xyz[:, :2] - lo) / res).astype(int)
        inside = (
            (cell[:, 0] >= 0)
            & (cell[:, 0] < width)
            & (cell[:, 1] >= 0)
            & (cell[:, 1] < height)
            & np.all(xyz[:, :2] <= np.asarray(hi), axis=1)
        )
        if int((np.abs(xyz[:, 2]) <= t["ground_abs_z_m"]).sum()) >= 10:
            visibility |= observed_height_bands(
                xyz, f["map_sensor_origin"], region, res, width, height
            )
        ground = inside & (np.abs(xyz[:, 2]) <= t["ground_abs_z_m"])
        obstacle = (
            inside
            & (xyz[:, 2] >= t["obstacle_min_z_m"])
            & (xyz[:, 2] <= t["obstacle_max_z_m"])
        )
        known = np.zeros(len(xyz), dtype=bool)
        for box in known_facilities:
            known |= np.all(
                (xyz >= np.array(box["min"]) - 0.1)
                & (xyz <= np.array(box["max"]) + 0.1),
                axis=1,
            )
        np.add.at(free_counts, (cell[ground, 1], cell[ground, 0]), 1)
        np.add.at(occ_counts, (cell[obstacle, 1], cell[obstacle, 0]), 1)
        np.add.at(
            known_counts, (cell[obstacle & known, 1], cell[obstacle & known, 0]), 1
        )
    occupied = set(
        zip(*np.where(occ_counts >= t["minimum_obstacle_points_per_cell"])[::-1])
    )
    accumulated_visibility = visibility.copy()
    if previous:
        if previous["region"] != region or previous["thresholds"] != t:
            raise ValueError("Cannot combine different regions/thresholds")
        accumulated_visibility |= np.asarray(
            previous["accumulated_height_band_mask"], dtype=np.uint8
        )
    free = set(zip(*np.where(accumulated_visibility == 7)[::-1])) - occupied
    occupied = {tuple(map(int, p)) for p in occupied}
    free = {tuple(map(int, p)) for p in free}
    old = set()
    if previous:
        if previous["region"] != region or previous["thresholds"] != t:
            raise ValueError("Cannot combine different regions/thresholds")
        old = {tuple(p) for p in previous["free_cells"] + previous["occupied_cells"]}
        occupied |= {tuple(p) for p in previous["occupied_cells"]}
        free |= {tuple(p) for p in previous["free_cells"]}
        free -= occupied
        valid = valid and previous["evidence_valid"]
    transform = lambda cells: (
        {(y, x) for x, y in cells} if region["passage_axis"] == 0 else cells
    )
    w, h = (height, width) if region["passage_axis"] == 0 else (width, height)
    outcome = classify_passage(
        width=w,
        height=h,
        observed_free=transform(free),
        occupied=transform(occupied),
        clearance_cells=math.ceil((t["clearance_radius_m"] + res / math.sqrt(2)) / res),
        minimum_coverage=t["minimum_coverage"],
        evidence_valid=valid,
    )
    extras = int(
        np.count_nonzero(
            (occ_counts - known_counts) >= t["minimum_obstacle_points_per_cell"]
        )
    )
    return {
        **outcome,
        "region": region,
        "thresholds": dict(t),
        "free_cells": sorted(map(list, free)),
        "occupied_cells": sorted(map(list, occupied)),
        "extra_obstacle_cells_this_observation": extras,
        "extra_obstacle_detected": extras > 0
        or bool(previous and previous["extra_obstacle_detected"]),
        "new_observed_cells": len((free | occupied) - old),
        "previous_observed_cells": len(old),
        "height_scope_m": [t["obstacle_min_z_m"], t["obstacle_max_z_m"]],
        "coverage_source": "finite-return rays in all three height bands (.10-.30/.30-.55/.55-.80m); no USD/costmap/predicted credit",
        "measured_ground_endpoint_cells_this_view": int(
            np.count_nonzero(free_counts >= t["minimum_ground_points_per_cell"])
        ),
        "height_band_mask_this_view": visibility.tolist(),
        "accumulated_height_band_mask": accumulated_visibility.tolist(),
        "limits": [
            "Unobserved cells remain unknown",
            "Low objects below declared band are not cleared",
            "No RGB recognition, identity or goods-condition inference",
        ],
    }


def inspect_clouds(
    snapshot, region, *, thresholds=None, previous=None, known_facilities=()
):
    """Include measured surroundings in full-footprint clearance at ROI boundaries.

    The passage is a robot-center path across the declared ROI. Rays outside it
    supply the required footprint halo; unknown halo cannot support CLEAR.
    Occupancy and coverage reported for the ROI remain raw measured cells.
    """
    from scipy.ndimage import maximum_filter, minimum_filter

    t = thresholds or DEFAULT_THRESHOLDS
    roi = _inspect_grid(
        snapshot,
        region,
        thresholds=t,
        previous=previous,
        known_facilities=known_facilities,
    )
    res = t["resolution_m"]
    radius = math.ceil((t["clearance_radius_m"] + res / math.sqrt(2)) / res)
    if t["clearance_radius_m"] == 0:
        return roi
    # Align halo with ROI grid, including partial final cells conservatively.
    lo, hi = np.asarray(region["min"]), np.asarray(region["max"])
    size = np.ceil((hi - lo) / res).astype(int)
    halo_region = {
        **region,
        "min": (lo - radius * res).tolist(),
        "max": (lo + (size + radius) * res).tolist(),
    }
    halo = _inspect_grid(
        snapshot,
        halo_region,
        thresholds=t,
        previous=previous.get("clearance_halo") if previous else None,
        known_facilities=known_facilities,
    )
    shape = (int(size[1] + 2 * radius), int(size[0] + 2 * radius))
    free, occupied = np.zeros(shape, bool), np.zeros(shape, bool)
    for x, y in halo["free_cells"]:
        free[y, x] = True
    for x, y in halo["occupied_cells"]:
        occupied[y, x] = True
    stencil = 2 * radius + 1
    safe = minimum_filter(free, size=stencil, mode="constant", cval=0)
    blocked = maximum_filter(occupied, size=stencil, mode="constant", cval=0)
    crop = (slice(radius, radius + size[1]), slice(radius, radius + size[0]))
    safe, blocked = safe[crop].copy(), blocked[crop]
    # Entire footprint must stay within the declared lateral passage boundary.
    if region["passage_axis"] == 1:
        safe[:, :radius] = False
        safe[:, max(0, size[0] - radius) :] = False
    else:
        safe[:radius, :] = False
        safe[max(0, size[1] - radius) :, :] = False
    cells = lambda mask: {tuple(map(int, p)) for p in zip(*np.where(mask)[::-1])}
    safe_cells, blocked_cells = cells(safe), cells(blocked)
    transform = lambda pts: (
        {(y, x) for x, y in pts} if region["passage_axis"] == 0 else pts
    )
    w, h = (
        (int(size[1]), int(size[0]))
        if region["passage_axis"] == 0
        else tuple(map(int, size))
    )
    configuration = classify_passage(
        width=w,
        height=h,
        observed_free=transform(safe_cells),
        occupied=transform(blocked_cells),
        clearance_cells=0,
        minimum_coverage=1e-12,
        evidence_valid=roi["evidence_valid"] and halo["evidence_valid"],
    )
    result = configuration["result"]
    reason = configuration["reason"]
    if result == "CLEAR" and roi["coverage_ratio"] < t["minimum_coverage"]:
        result, reason = "UNKNOWN", "ROI measured coverage below frozen threshold"
    return {
        **roi,
        "result": result,
        "reason": "Full-footprint measured halo: " + reason,
        "verified_free_path": result == "CLEAR",
        "measured_obstacle_barrier": configuration["measured_obstacle_barrier"],
        "clearance_halo": halo,
        "boundary_obstacle_cells": halo["obstacle_cells"],
        "halo_coverage_ratio": halo["coverage_ratio"],
        "clearance_blocked_center_count": len(blocked_cells),
        "clearance_safe_center_cells": sorted(map(list, safe_cells)),
        "clearance_blocked_center_cells": sorted(map(list, blocked_cells)),
        "boundary_clearance_source": "Actual raw PointCloud2 halo only; no prior/ground-truth free or occupied credit",
    }
