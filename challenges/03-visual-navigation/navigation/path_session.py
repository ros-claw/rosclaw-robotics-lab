"""Order plan and action replies for one committed navigation route."""

import math


class PathSession:
    route_version = 1

    def __init__(self):
        self.latest = 0
        self.active = None
        self.terminated = False

    def request(self):
        if self.terminated:
            raise ValueError("path session has terminated")
        self.latest += 1
        return self.latest

    def may_submit(self, path_seq):
        return (not self.terminated and 0 < path_seq == self.latest
                and (self.active is None or self.active < path_seq))

    def accept(self, path_seq):
        if not self.may_submit(path_seq):
            return False
        self.active = path_seq
        return True

    def finish(self, path_seq):
        if self.terminated or path_seq != self.active or path_seq is None:
            return False
        self.terminated = True
        self.active = None
        return True


def validate_handoff(points, position, yaw):
    """Find a local forward join and reject a detached or reversing replacement."""
    if not all(math.isfinite(value) for value in (*position, yaw)):
        raise ValueError("handoff pose is invalid")
    distance_ahead = 0.0
    best = None
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        limit = min(length, 0.45 - distance_ahead)
        projection = ((position[0] - start[0]) * dx + (position[1] - start[1]) * dy) / length
        along = min(limit, max(0.0, projection)) / length
        join = (start[0] + dx * along, start[1] + dy * along)
        error = math.dist(position, join)
        if best is None or error < best[0]:
            best = error, join, math.remainder(math.atan2(dy, dx) - yaw, math.tau)
        distance_ahead += length
        if distance_ahead >= 0.45:
            break
    if best is None or best[0] > 0.20 or abs(best[2]) > 0.9:
        raise ValueError("replacement path cannot be joined smoothly from the current pose")
    return best[1]
