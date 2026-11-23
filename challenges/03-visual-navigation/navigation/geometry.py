"""Projection for a rectilinear camera with optical-axis depth in meters."""

from dataclasses import dataclass
import math

from .contracts import ObservationRejected


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if (self.width < 1 or self.height < 1 or not all(
            math.isfinite(value) for value in (self.fx, self.fy, self.cx, self.cy)
        ) or min(self.fx, self.fy) <= 0):
            raise ObservationRejected("invalid camera calibration")

    def _check_pixel(self, u, v):
        if not 0 <= u < self.width or not 0 <= v < self.height:
            raise ObservationRejected("pixel is outside the image")

    def unproject(self, u, v, depth_m):
        self._check_pixel(u, v)
        if not math.isfinite(depth_m) or depth_m <= 0:
            raise ObservationRejected("depth must be positive and finite")
        return (u - self.cx) * depth_m / self.fx, (v - self.cy) * depth_m / self.fy, depth_m

    def project(self, optical_xyz):
        x, y, z = optical_xyz
        if not all(map(math.isfinite, optical_xyz)) or z <= 0:
            raise ObservationRejected("point must be finite and in front of the camera")
        u, v = self.fx * x / z + self.cx, self.fy * y / z + self.cy
        self._check_pixel(u, v)
        return u, v
