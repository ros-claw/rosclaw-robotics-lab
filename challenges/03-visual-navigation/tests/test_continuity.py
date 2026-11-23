from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluator.continuity import motion_window


def row(seconds, speed):
    return {"source": "physx", "sim_time_s": seconds,
            "host_mono_ns": str(round(seconds * 1e9)),
            "linear_angular_velocity": [speed, 0, 0, 0, 0, 0]}


def test_handoff_needs_motion_and_contiguous_samples_on_both_sides():
    samples = [row(k / 20, 0.2) for k in range(40)]
    assert motion_window(samples, 1_000_000_000)["pass"]
    assert not motion_window(samples[:20], 1_000_000_000)["pass"]
    reordered = samples[:]
    reordered[18], reordered[19] = reordered[19], reordered[18]
    assert not motion_window(reordered, 1_000_000_000)["pass"]
    samples[20] = row(1.0, 0.01)
    assert not motion_window(samples, 1_000_000_000)["pass"]
    del samples[19:23]
    assert not motion_window(samples, 1_000_000_000)["pass"]


def test_independent_gate_check_requires_the_full_footprint_inside():
    from evaluator.continuity import minimum_corridor_clearance
    from navigation.candidates import Gate

    gate = Gate("right", 0, -3.9, -1.0, -0.3, 2.3)
    inside = [{"pose_xyz_xyzw": [0, -2.5, 0]}]
    outside = [{"pose_xyz_xyzw": [0, -1.4, 0]}]
    assert minimum_corridor_clearance(inside, gate, 0.65) > 0
    assert minimum_corridor_clearance(outside, gate, 0.65) < 0
