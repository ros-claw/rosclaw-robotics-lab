"""Bounded calibrated SIM inspection profile; never an authorization expansion."""

import math


def validate_thresholds(values):
    required = {
        "resolution_m",
        "ground_abs_z_m",
        "obstacle_min_z_m",
        "obstacle_max_z_m",
        "minimum_ground_points_per_cell",
        "minimum_obstacle_points_per_cell",
        "minimum_coverage",
        "clearance_radius_m",
        "max_observations",
        "minimum_gain_cells",
    }
    if not isinstance(values, dict) or set(values) != required:
        raise ValueError("Complete, exact calibrated inspection profile required")
    for key, value in values.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("Finite numeric inspection profile required: " + key)
    for key in (
        "minimum_ground_points_per_cell",
        "minimum_obstacle_points_per_cell",
        "max_observations",
        "minimum_gain_cells",
    ):
        if type(values[key]) is not int or values[key] < 1:
            raise ValueError("Positive integer inspection budget/count required")
    if values["max_observations"] != 2 or values["minimum_gain_cells"] < 3:
        raise ValueError(
            "At most two views; active observation needs >=3 measured new cells"
        )
    if (
        not 0.05 <= values["resolution_m"] <= 0.5
        or not 0.70 <= values["minimum_coverage"] <= 1
    ):
        raise ValueError("Calibrated grid/coverage bounds required")
    if (
        not 0 < values["ground_abs_z_m"] <= 0.08
        or values["obstacle_min_z_m"] != 0.10
        or values["obstacle_max_z_m"] != 0.80
    ):
        raise ValueError("Only audited ground/obstacle height bands are supported")
    if not 0.75 <= values["clearance_radius_m"] <= 2:
        raise ValueError(
            "Declared Body clearance cannot be reduced below calibrated 0.75m"
        )
    return dict(values)
