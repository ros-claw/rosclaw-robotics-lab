from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.luna_probe import observation_id
from navigation.reobserve import FreshFrames


def make_observation(stamp=100, episode="run", epoch=0):
    return {"episode_id": episode, "epoch": epoch, "stamp_ns": str(stamp), "frame_seq": 1,
            "localization_revision": 0,
            "transforms": [{"target": "map", "source": "odom", "stamp_ns": str(stamp), "xyz_m": [0, 0, 0],
                            "quaternion_xyzw": [0, 0, 0, 1]}]}


def test_restarted_capture_uses_the_actual_image_stamp_for_identity():
    assert observation_id(make_observation(100)) != observation_id(make_observation(101))


def test_distinct_capture_must_advance_without_crossing_epoch_or_localization():
    guard = FreshFrames("run")
    guard.check(make_observation())
    for observation in (make_observation(99), make_observation(), make_observation(101, "old"),
                        make_observation(101, epoch=1),
                        {**make_observation(101), "localization_revision": 1}):
        with pytest.raises(ValueError):
            guard.check(observation)
    guard.check(make_observation(101))



def test_reobservation_requires_stamped_localization_and_rejects_a_pose_jump():
    guard = FreshFrames("run")
    with pytest.raises(ValueError):
        guard.check({**make_observation(), "transforms": []})
    guard.check(make_observation())
    shifted = make_observation(101)
    shifted["transforms"][0]["xyz_m"][0] = 0.25
    with pytest.raises(ValueError):
        guard.check(shifted)
