"""Warehouse known-prior geometry. No random obstacle truth or split in Agent view."""

import math
from pathlib import PurePosixPath
import numpy as np


def aabb_distance(a, b):
    return math.hypot(
        *(max(a["min"][i] - b["max"][i], b["min"][i] - a["max"][i], 0) for i in (0, 1))
    )


def entities(audit):
    rows = audit["rows"]
    result = []
    for category, terms in [
        ("forklift", ("forklift",)),
        ("shelf", ("rackshelf",)),
        ("cart", ("pushcart", "trolley", "trailer")),
        ("pallet", ("palette", "pallet")),
    ]:
        matches = [
            r
            for r in rows
            if r["type"] == "Xform"
            and any(
                (
                    PurePosixPath(r["path"]).name.lower().startswith(t)
                    if category == "forklift"
                    else t in PurePosixPath(r["path"]).name.lower()
                )
                for t in terms
            )
        ]
        for r in matches:
            if any(r["path"].startswith(p["path"] + "/") for p in matches if p != r):
                continue
            result.append(
                {
                    **{k: r[k] for k in ("path", "min", "max")},
                    "category": category,
                    "colliders": [
                        {k: c[k] for k in ("path", "min", "max")}
                        for c in rows
                        if c["path"].startswith(r["path"] + "/")
                        and c["collision_enabled"]
                    ],
                }
            )
    return result


def relations(objects, ambiguity_margin=0.25):
    shelves = [r for r in objects if r["category"] == "shelf"]
    forks = [r for r in objects if r["category"] == "forklift"]
    if not shelves or len(forks) < 2:
        raise ValueError("Need measured shelves and at least two forklifts")
    rows = []
    for f in forks:
        shelf = min(shelves, key=lambda s: aabb_distance(f, s))
        rows.append(
            {
                "target": f,
                "nearest_shelf": shelf,
                "shelf_distance_m": aabb_distance(f, shelf),
            }
        )
    rows.sort(key=lambda r: (r["shelf_distance_m"], r["target"]["path"]))
    ambiguous = (
        rows[1]["shelf_distance_m"] - rows[0]["shelf_distance_m"] < ambiguity_margin
    )
    return {
        "status": "AMBIGUOUS" if ambiguous else "RESOLVED",
        "ranked": rows,
        "method": "minimum planar distance between measured object bounds; no robot-distance or name suffix selection",
        "ambiguity_margin_m": ambiguity_margin,
    }


def region_for_relation(relation):
    """Ground work patch alongside near-shelf vehicle, front face inferred geometrically.

    Fixed declared construction rules are parameters, not industrial standards.
    Passage direction is local y. Regions do not imply robot entry is safe.
    """
    f, s = relation["target"], relation["nearest_shelf"]
    center = [(s["min"][i] + s["max"][i]) / 2 for i in (0, 1)]
    fc = [(f["min"][i] + f["max"][i]) / 2 for i in (0, 1)]
    axis = 0 if abs(fc[0] - center[0]) >= abs(fc[1] - center[1]) else 1
    sign = 1 if fc[axis] >= center[axis] else -1
    lateral = 1 - axis
    face = s["max"][axis] if sign > 0 else s["min"][axis]
    outer = f["max"][axis] + 1.0 if sign > 0 else f["min"][axis] - 1.0
    if abs(outer - face) > 5.0:
        return None
    lo = [0.0, 0.0]
    hi = [0.0, 0.0]
    lo[axis], hi[axis] = sorted([face + sign * 0.3, outer])
    lo[lateral] = f["max"][lateral] + 0.3
    hi[lateral] = lo[lateral] + 2.8
    return {
        "min": lo,
        "max": hi,
        "passage_axis": lateral,
        "frame": "map",
        "source": {
            "shelf": s["path"],
            "forklift": f["path"],
            "method": "shelf front to forklift outer bound +1m; adjacent 2.8m work patch; 0.3m facility offset",
        },
        "scope": "ground-height passage for this robot; no full warehouse/industrial safety claim",
    }


def point_in_box(xy, box, margin=0):
    return all(
        box["min"][i] - margin <= xy[i] <= box["max"][i] + margin for i in (0, 1)
    )


def observation_seeds(region):
    lo, hi = region["min"], region["max"]
    cx, cy = [(a + b) / 2 for a, b in zip(lo, hi)]
    # Both sides of a geometric work patch, no named-site coordinates.
    return (
        [(hi[0] + 2.3, lo[1] - 4.5)]
        + [(hi[0] + d, cy + dy) for d in (1.6, 2.3) for dy in (-1.0, 0.0, 1.0)]
        + [(cx + dx, hi[1] + d) for d in (1.6, 2.3) for dx in (0.0, 1.5)]
    )


def add_exclusions(grid, boxes):
    """Copy grid; mark actual known facility envelopes for independent footprint gates."""
    data = (
        np.asarray(grid["data"], dtype=int)
        .reshape(grid["height"], grid["width"])
        .copy()
    )
    res = grid["resolution"]
    origin = grid["origin"]
    for box in boxes:
        lo = np.floor((np.array(box["min"][:2]) - origin) / res).astype(int)
        hi = np.floor((np.array(box["max"][:2]) - origin) / res).astype(int)
        x0, y0 = np.maximum(lo, 0)
        x1, y1 = np.minimum(hi, [grid["width"] - 1, grid["height"] - 1])
        if x0 <= x1 and y0 <= y1:
            data[y0 : y1 + 1, x0 : x1 + 1] = 100
    return {**grid, "data": data.ravel().tolist()}


def predicted_cells(pose, region, res=0.2):
    # Prior prediction only; NEVER counted as measured inspection coverage.
    lo, hi = region["min"], region["max"]
    result = []
    for y in range(int((hi[1] - lo[1]) / res)):
        for x in range(int((hi[0] - lo[0]) / res)):
            point = [lo[0] + (x + 0.5) * res, lo[1] + (y + 0.5) * res]
            if 2.1 <= math.dist(point, pose[:2]) <= 12:
                result.append([x, y])
    return result
