"""Bound reactions to route health independently of the scene and its obstacle actors."""

from collections import Counter
import math

ROUTE_UNAVAILABLE = 42
ROUTE_STOPPED = 43


class RouteUnavailable(ValueError):
    pass


class RouteBlocked(RuntimeError):
    pass


class Blockage:
    def __init__(self, dwell_sim_s=0.65):
        if not math.isfinite(dwell_sim_s) or dwell_sim_s <= 0:
            raise ValueError("invalid blockage dwell threshold")
        self.minimum_ns = round(dwell_sim_s * 1e9)
        self.stamp = None
        self.first_blocked = None

    def observe(self, stamp_ns, blocked):
        if self.stamp is not None and stamp_ns <= self.stamp:
            return False
        self.stamp = stamp_ns
        if not blocked:
            self.first_blocked = None
            return False
        if self.first_blocked is None:
            self.first_blocked = stamp_ns
        return stamp_ns - self.first_blocked >= self.minimum_ns


class Recovery:
    def __init__(self, local_retries, allow_alternate):
        self.local_retries = local_retries
        self.allow_alternate = allow_alternate
        self.attempts = Counter()
        self.excluded = set()

    def after_stop(self, gate):
        if self.attempts[gate] < self.local_retries:
            self.attempts[gate] += 1
            return "retry"
        self.excluded.add(gate)
        return "decide" if self.allow_alternate else "hold"


def path_ahead(points, position, lookahead):
    nearest = min(range(len(points)), key=lambda index: math.dist(position, points[index]))
    previous = points[nearest]
    distance = math.dist(position, previous)
    for point in points[nearest + 1:]:
        distance += math.dist(previous, point)
        if distance > lookahead:
            break
        yield point
        previous = point


def alignment_turn(points, position, yaw):
    ahead = list(path_ahead(points, position, 0.9))
    if not ahead:
        raise RouteUnavailable("no usable path segment for alignment")
    x, y = ahead[-1]
    return math.remainder(math.atan2(y - position[1], x - position[0]) - yaw, math.tau)


def blocked_ahead(path, position, lookahead, radius, is_free):
    return next((point for point in path_ahead(path, position, lookahead)
                 if not is_free(*point, radius)), None)
