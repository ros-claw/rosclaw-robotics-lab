from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.adaptation import Blockage, Recovery, path_ahead


def test_only_fresh_persistent_map_observations_trigger_recovery():
    monitor = Blockage()
    assert not monitor.observe(1_000_000_000, True)
    assert not monitor.observe(1_000_000_000, True)
    assert not monitor.observe(2_000_000_000, False)
    assert not monitor.observe(3_000_000_000, True)
    assert monitor.observe(4_000_000_000, True)


def test_local_recovery_preserves_semantics_and_bounds_retry_count():
    recovery = Recovery(2, allow_alternate=True)
    assert recovery.after_stop("dock") == "retry"
    assert recovery.after_stop("dock") == "retry"
    assert recovery.after_stop("dock") == "decide"
    assert recovery.excluded == {"dock"}
    assert recovery.after_stop("other") == "retry"
    strict = Recovery(0, allow_alternate=False)
    assert strict.after_stop("dock") == "hold"


def test_obstacles_behind_or_far_ahead_are_outside_the_active_window():
    line = [(float(x), 0.0) for x in range(9)]
    assert list(path_ahead(line, (4.1, 0), 2.5)) == [(5., 0.), (6., 0.)]


def test_bounded_alignment_uses_the_new_paths_local_heading():
    import math
    from navigation.adaptation import alignment_turn

    path = [(0, 0), (0, -0.3), (0, -0.6), (0, -0.9)]
    assert math.isclose(alignment_turn(path, (0, 0), 0), -math.pi / 2)
    assert math.isclose(alignment_turn(path, (0, 0), -math.pi / 2), 0)
