"""Neutral gate geometry and conservative swept-footprint route checks."""

from dataclasses import dataclass
import json
import math


class ValidationError(ValueError):
    pass


@dataclass(frozen=True)
class Gate:
    gate_id: str
    cross_x: float
    lower_y: float
    upper_y: float
    entry_x: float
    exit_x: float

    def __post_init__(self):
        edges = (self.cross_x, self.lower_y, self.upper_y, self.entry_x, self.exit_x)
        if (not all(map(math.isfinite, edges)) or self.lower_y >= self.upper_y
                or not self.entry_x < self.cross_x < self.exit_x):
            raise ValidationError("invalid gate geometry")


def costmap_footprint_radius(params):
    costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
    footprint = json.loads(costmap["footprint"])
    padding = float(costmap.get("footprint_padding", 0.01))
    return max(math.hypot(x, y) for x, y in footprint) + math.sqrt(2.0) * padding


def crossings(path, plane_x):
    """Return directed center-line crossings without duplicating shared endpoints."""
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        if x0 < plane_x <= x1 or x1 < plane_x <= x0:
            fraction = (plane_x - x0) / (x1 - x0)
            yield x1 > x0, y0 + fraction * (y1 - y0)


def validate_corridor(path, chosen, forbidden, radius, is_open):
    """Require the committed forward gate and reject any later exit from its corridor."""
    if len(path) < 2 or not math.isfinite(radius) or radius <= 0:
        raise ValidationError("path and robot footprint are required")
    if any(len(point) != 2 or not all(map(math.isfinite, point)) for point in path):
        raise ValidationError("path must contain finite map-frame XY points")

    for gate in forbidden:
        for _, y in crossings(path, gate.cross_x):
            if gate.lower_y - radius <= y <= gate.upper_y + radius:
                raise ValidationError(f"path crosses forbidden gate {gate.gate_id}")

    selected = list(crossings(path, chosen.cross_x))
    if len(selected) != 1 or not selected[0][0]:
        raise ValidationError("path must cross the committed gate once in the forward direction")
    if path[-1][0] < chosen.exit_x:
        raise ValidationError("path ends within the committed corridor")

    samples_max_step = 0.05
    padded = radius + samples_max_step / 2
    for start, end in zip(path, path[1:]):
        ticks = max(1, math.ceil(math.dist(start, end) / samples_max_step))
        for index in range(ticks + 1):
            fraction = index / ticks
            x, y = (a + fraction * (b - a) for a, b in zip(start, end))
            if chosen.entry_x - padded <= x <= chosen.exit_x + padded:
                if not chosen.lower_y + padded <= y <= chosen.upper_y - padded:
                    raise ValidationError("footprint exits the committed corridor")
            if not is_open(x, y, padded):
                raise ValidationError(f"path has insufficient footprint clearance near ({x:.3f}, {y:.3f})")
