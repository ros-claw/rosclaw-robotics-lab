from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.live_intent import verify_fresh


def test_live_intent_is_bound_to_time_epoch_and_stationary_pose():
    q = [0, 0, 0, 1]
    obs = {"episode_id": "run", "epoch": 0, "stamp_ns": "1000000000",
           "rgb_receive_mono_ns": "10000000000", "diagnostic_camera_yaw_deg": 0,
           "transforms": [{"target": "map", "source": "base_link",
                           "xyz_m": [-4.5, 0, 0], "quaternion_xyzw": q}]}

    def verify(episode="run", clock=4_000_000_000, mono=13_000_000_000, pose=(-4.5, 0, 0)):
        verify_fresh(obs, episode, clock, mono, pose, q)

    verify()
    for kwargs in ({"episode": "other"}, {"clock": 900_000_000}, {"mono": 18_100_000_000},
                   {"pose": (-4.2, 0, 0)}, {"pose": (float("nan"), 0, 0)}):
        with pytest.raises(ValueError):
            verify(**kwargs)
