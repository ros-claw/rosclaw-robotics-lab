from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.path_session import PathSession, validate_handoff


def test_new_request_retires_held_plan_and_old_goal_result():
    session = PathSession()
    assert not session.accept(0)
    first = session.request()
    assert session.accept(first)
    delayed = session.request()
    second = session.request()
    assert not session.accept(delayed)
    assert session.accept(second)
    assert not session.finish(first)
    assert session.active == second and session.route_version == 1
    third = session.request()
    assert session.accept(third)
    assert not session.finish(second)
    assert session.finish(third)
    assert not session.finish(third)
    with pytest.raises(ValueError, match="terminated"):
        session.request()


def test_current_result_is_authoritative_while_replacement_is_pending():
    session = PathSession()
    first = session.request()
    assert session.accept(first)
    second = session.request()
    assert session.finish(first)
    assert not session.accept(second)


def test_handoff_rejects_reversed_or_disconnected_paths():
    points = [(0.0, 0.0), (0.2, 0.0), (0.6, 0.0), (1.0, 0.0)]
    validate_handoff(points, (0.12, 0.06), 0.05)
    for position, yaw in (((0.1, 0.4), 0.0), ((0.1, 0.0), 3.1),
                          ((0.85, 0.0), 0.0), ((float("nan"), 0.1), 0.0)):
        with pytest.raises(ValueError):
            validate_handoff(points, position, yaw)
