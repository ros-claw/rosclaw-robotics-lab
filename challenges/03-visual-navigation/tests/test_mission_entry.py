from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.run_mission import mission_status


def test_mission_reports_hold_separately_and_never_guesses_success():
    assert mission_status(0, {"event": "hold", "reason": "obstacle"}) == ("hold", "obstacle")
    assert mission_status(0, {"event": "route_completed"}) == ("succeeded", None)
    assert mission_status(2, {"event": "route_completed"}) == ("failed", "controller_exit")
    assert mission_status(0, {}) == ("failed", "missing_terminal_evidence")
