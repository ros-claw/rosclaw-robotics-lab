"""Fixed Nova Carter camera mount, from base_link to ROS optical axes."""

import math

BASE_FRAME = "base_link"
OPTICAL_FRAME = "luna_front_optical"
TRANSLATION_M = (0.45, 0.0, 0.52)
OPTICAL_XYZW = (-0.5, 0.5, -0.5, 0.5)
WIDTH, HEIGHT = 960, 540
TOPICS = {"rgb": "/luna/front/rgb", "depth": "/luna/front/depth",
          "info": "/luna/front/camera_info"}


def yaw_mount_xyzw(quaternion, yaw_degrees):
    """Rotate the fixed mount about the chassis vertical axis."""
    x, y, z, w = quaternion
    half_angle = math.radians(yaw_degrees) / 2
    cosine, sine = math.cos(half_angle), math.sin(half_angle)
    return (cosine * x - sine * y, sine * x + cosine * y,
            cosine * z + sine * w, cosine * w - sine * z)
