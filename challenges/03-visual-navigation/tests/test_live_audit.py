import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.live_audit import review_is_current


def test_review_is_bound_to_active_goal_camera_pose_and_fresh_time():
    review = {"episode_id": "episode", "goal_id": "goal", "gate": "a",
              "sim_stamp_ns": 10_000_000_000, "frame_wall_ns": 40_000_000_000,
              "reply_wall_ns": 43_000_000_000, "base_xy": [0.0, 0.0], "base_yaw": 0.0}
    def verify(goal="goal", now=44_000_000_000, position=(1.0, 0.0), yaw=0.2):
        return review_is_current(review, "episode", goal, "a", 14_000_000_000,
                                 now, position, yaw)
    assert verify()
    assert not verify(goal="previous")
    assert not verify(now=55_000_000_000)
    assert not verify(position=(1.6, 0.0))
    assert verify(yaw=1.0)
    assert not verify(yaw=1.25)


def test_motion_reply_cannot_select_a_route():
    import json
    import pytest
    from navigation.luna_probe import AUDIT_DECISIONS, ProbeError, parse_intent

    reply = {"observation_id": "frame", "decision": "review", "candidate_id": None,
             "reason_summary": "visible barrier", "evidence_regions": []}

    def raw(intent):
        return {"status": "completed", "output": [{"type": "message", "role": "assistant",
                 "content": [{"type": "output_text", "text": json.dumps(intent)}]}]}

    assert parse_intent(raw(reply), "frame", {}, AUDIT_DECISIONS)["decision"] == "review"
    with pytest.raises(ProbeError):
        parse_intent(raw({**reply, "decision": "select_candidate", "candidate_id": "gate"}),
                     "frame", {}, AUDIT_DECISIONS)


def test_active_goal_is_unavailable_as_soon_as_its_cancel_starts(tmp_path):
    import json
    from navigation.live_audit import active_goal

    path = tmp_path / "controller.jsonl"
    with path.open("w") as stream:
        stream.write(json.dumps({"event": "nav2_accepted", "goal_id": "goal"}) + "\n")
    assert active_goal(path) == "goal"
    with path.open("a") as stream:
        stream.write(json.dumps({"event": "cancel_requested"}) + "\n")
    assert active_goal(path) is None
