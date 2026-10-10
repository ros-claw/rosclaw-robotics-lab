"""Invariants for relation selection, measured coverage and bounded report semantics."""

import base64
import hashlib
import struct
import sys
from pathlib import Path
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from loading_geometry import relations, entities
from loading_perception import inspect_clouds


def object_(path, category, lo, hi):
    return {"path": path, "category": category, "min": lo, "max": hi, "colliders": []}


def test_forklift_relation_ignores_names_and_robot_distance():
    objects = [
        object_("/shelf", "shelf", [0, 0, 1], [1, 4, 3]),
        object_("/Forklift", "forklift", [1, 0, 0], [3, 1, 2]),
        object_("/Forklift_01", "forklift", [10, 0, 0], [12, 1, 2]),
    ]
    out = relations(objects)
    assert out["ranked"][0]["target"]["path"] == "/Forklift"
    assert out["status"] == "RESOLVED"


def test_close_distances_are_ambiguous():
    objects = [
        object_("/shelf", "shelf", [0, 0, 1], [1, 4, 3]),
        object_("/a", "forklift", [2, 0, 0], [3, 1, 2]),
        object_("/b", "forklift", [2.1, 2, 0], [3.1, 3, 2]),
    ]
    assert relations(objects)["status"] == "AMBIGUOUS"


def test_container_name_is_not_forklift_entity():
    audit = {
        "rows": [
            {
                "path": "/warehouse_with_forklifts",
                "type": "Xform",
                "min": [0] * 3,
                "max": [10] * 3,
                "collision_enabled": False,
            },
            {
                "path": "/warehouse_with_forklifts/Forklift",
                "type": "Xform",
                "min": [0] * 3,
                "max": [1] * 3,
                "collision_enabled": False,
            },
        ]
    }
    assert len(entities(audit)) == 1


def snapshot(points):
    raw = b"".join(struct.pack("<ddd", *p) for p in points)
    return {
        "status": "PASS",
        "frames": [
            {
                "stamp": 1.0,
                "clock": 1.01,
                "tf_stamp": 1.0,
                "points_map": points,
                "finite_points": len(points),
                "decoded_points": len(points),
                "width": len(points),
                "height": 1,
                "row_step": len(raw),
                "point_step": 24,
                "is_bigendian": False,
                "fields": [
                    {"name": k, "offset": i * 8, "datatype": 8, "count": 1}
                    for i, k in enumerate(("x", "y", "z"))
                ],
                "map_sensor_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "map_sensor_origin": [0, 0, 0],
                "raw_data_base64": base64.b64encode(raw).decode(),
                "raw_data_sha256": hashlib.sha256(raw).hexdigest(),
            }
        ],
    }


REGION = {"min": [0, 0], "max": [2, 2], "passage_axis": 1}


def test_high_points_cannot_clear_floor():
    out = inspect_clouds(snapshot([[0.2, 0.2, 4.0], [0.2, 0.2, 4.0]]), REGION)
    assert out["result"] == "UNKNOWN"
    assert out["observed_cells"] == 0


def test_stale_tf_is_unknown():
    snap = snapshot([[0.2, 0.2, 0]] * 3)
    snap["frames"][0]["tf_stamp"] = 2
    out = inspect_clouds(snap, REGION)
    assert not out["evidence_valid"] and out["result"] == "UNKNOWN"


def test_extra_obstacle_is_separate_from_blockage():
    points = [[0.5, 0.5, 0.3]] * 5
    out = inspect_clouds(snapshot(points), REGION)
    assert out["extra_obstacle_detected"]
    assert out["result"] == "UNKNOWN"


def test_coverage_only_increases_from_measured_new_cells():
    first = inspect_clouds(snapshot([[0.5, 0.5, 0.3]] * 3), REGION)
    same = inspect_clouds(snapshot([[0.5, 0.5, 0.3]] * 3), REGION, previous=first)
    second = inspect_clouds(snapshot([[1.5, 1.5, 0.3]] * 3), REGION, previous=first)
    assert same["new_observed_cells"] == 0
    assert second["new_observed_cells"] == 1


def test_cannot_merge_different_regions():
    first = inspect_clouds(snapshot([[0.5, 0.5, 0.3]] * 3), REGION)
    with pytest.raises(ValueError):
        inspect_clouds(
            snapshot([[0.5, 0.5, 0]] * 3), {**REGION, "max": [3, 3]}, previous=first
        )


def test_previous_extra_obstacle_not_forgotten():
    first = inspect_clouds(snapshot([[0.5, 0.5, 0.3]] * 3), REGION)
    second = inspect_clouds(snapshot([[1.5, 1.5, 0.3]] * 3), REGION, previous=first)
    assert second["extra_obstacle_detected"]


def test_raw_pointcloud_tamper_is_rejected():
    snap = snapshot([[0.5, 0.5, 0]] * 3)
    snap["frames"][0]["points_map"][0][0] = 1.0
    with pytest.raises(ValueError):
        inspect_clouds(snap, REGION)


def test_missing_raw_buffer_cannot_claim_inspection():
    snap = snapshot([[0.5, 0.5, 0]] * 3)
    del snap["frames"][0]["raw_data_base64"]
    with pytest.raises(ValueError):
        inspect_clouds(snap, REGION)


@pytest.mark.parametrize(
    "halo_state, expected",
    [("free", "CLEAR"), ("barrier", "OBSTRUCTED"), ("unknown", "UNKNOWN")],
)
def test_full_footprint_halo_is_required_at_roi_edge(monkeypatch, halo_state, expected):
    import loading_perception as perception

    # 3m ROI, 1m conservative clearance stencil, halo covers whole footprint.
    region = {"min": [0, 0], "max": [3, 3], "passage_axis": 1}
    roi_free = [[x, y] for x in range(15) for y in range(15)]
    halo_free = [[x, y] for x in range(25) for y in range(25)]
    halo_occ = []
    if halo_state == "barrier":
        # Actual obstacle is outside the ROI, within its entry footprint.
        halo_occ = [[x, 4] for x in range(25)]
        halo_free = [p for p in halo_free if p not in halo_occ]
    elif halo_state == "unknown":
        halo_free = [[x + 5, y + 5] for x, y in roi_free]

    def measured_grid(snapshot, box, **kwargs):
        is_roi = box == region
        return {
            "result": "CLEAR",
            "region": box,
            "free_cells": roi_free if is_roi else halo_free,
            "occupied_cells": [] if is_roi else halo_occ,
            "evidence_valid": True,
            "coverage_ratio": 1.0,
            "obstacle_cells": 0,
        }

    monkeypatch.setattr(perception, "_inspect_grid", measured_grid)
    report = perception.inspect_clouds({}, region)
    assert report["result"] == expected
    assert report["obstacle_cells"] == 0  # Raw ROI occupancy is not rewritten.
    assert report["coverage_ratio"] == 1.0  # Halo is not substituted for coverage.


@pytest.mark.parametrize(
    "key, value",
    [
        ("max_observations", 3),
        ("max_observations", True),
        ("minimum_gain_cells", 1),
        ("clearance_radius_m", 0.1),
        ("minimum_coverage", float("nan")),
        ("obstacle_min_z_m", 0.05),
    ],
)
def test_body_cannot_expand_budget_or_reduce_audited_safety(key, value):
    from loading_contract import validate_thresholds
    from loading_perception import DEFAULT_THRESHOLDS

    with pytest.raises(ValueError):
        validate_thresholds({**DEFAULT_THRESHOLDS, key: value})


def test_calibrated_contract_is_valid():
    from loading_contract import validate_thresholds
    from loading_perception import DEFAULT_THRESHOLDS

    assert validate_thresholds(DEFAULT_THRESHOLDS) == DEFAULT_THRESHOLDS


def test_actual_ray_does_not_clear_beyond_its_return_or_other_heights():
    import numpy as np
    from loading_visibility import observed_height_bands

    region = {"min": [0, -0.1], "max": [3, 0.1]}
    mask = observed_height_bands(np.array([[2, 0, 0.2]]), [0, 0, 0.2], region, 1, 3, 1)
    assert mask.tolist() == [[1, 1, 0]]
    missing = observed_height_bands(np.empty((0, 3)), [0, 0, 0.2], region, 1, 3, 1)
    assert not missing.any()
    high = observed_height_bands(np.array([[2, 0, 4.1]]), [0, 0, 4.1], region, 1, 3, 1)
    assert not high.any()
