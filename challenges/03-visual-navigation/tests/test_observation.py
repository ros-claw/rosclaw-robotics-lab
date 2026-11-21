import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.contracts import CaptureEpoch, ObservationRejected
from navigation.geometry import CameraIntrinsics


def test_epoch_reset_rejects_old_result():
    epoch = CaptureEpoch()
    epoch.observe_clock(200)
    first = epoch.accept_capture(175, 0)
    assert first.epoch == 0
    assert epoch.observe_clock(10)
    with pytest.raises(ObservationRejected, match="epoch"):
        epoch.accept_capture(9, first.epoch)


def test_pause_rejects_duplicate_capture():
    epoch = CaptureEpoch()
    epoch.observe_clock(200)
    epoch.accept_capture(175, 0)
    epoch.observe_clock(200)
    with pytest.raises(ObservationRejected, match="repeated"):
        epoch.accept_capture(175, 0)


@pytest.mark.parametrize("depth", [math.nan, math.inf, 0.0, -1.0])
def test_invalid_depth_cannot_produce_point(depth):
    intrinsics = CameraIntrinsics(960, 540, 500, 500, 480, 270)
    with pytest.raises(ObservationRejected, match="depth"):
        intrinsics.unproject(480, 270, depth)


def test_z_depth_round_trip():
    intrinsics = CameraIntrinsics(960, 540, 500, 500, 480, 270)
    optical = intrinsics.unproject(580, 170, 2.0)
    assert optical == (0.4, -0.4, 2.0)
    assert intrinsics.project(optical) == (580, 170)


def test_velocity_dwell_survives_clock_and_odom_callback_order():
    from navigation.contracts import StopDwell

    dwell = StopDwell()
    stamp = 1_000_000_000
    assert not dwell.observe(stamp, stamp, 0.0, 0.0)
    for _ in range(1, 4):
        stamp += 250_000_000
        assert not dwell.observe(stamp - 20_000_000, stamp, 0.0, 0.0)
        settled = dwell.observe(stamp, stamp, 0.0, 0.0)
    assert settled
