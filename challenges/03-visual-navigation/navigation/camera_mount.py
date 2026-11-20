"""Fixed Nova Carter camera mount, from base_link to ROS optical axes."""

BASE_FRAME = "base_link"
OPTICAL_FRAME = "luna_front_optical"
TRANSLATION_M = (0.45, 0.0, 0.52)
OPTICAL_XYZW = (-0.5, 0.5, -0.5, 0.5)
WIDTH, HEIGHT = 960, 540
TOPICS = {"rgb": "/luna/front/rgb", "depth": "/luna/front/depth",
          "info": "/luna/front/camera_info"}
