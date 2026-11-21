import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.luna_probe import ProbeError, parse_intent


def response(text, status="completed"):
    return {"status": status, "output": [{"type": "message", "role": "assistant",
                                          "content": [{"type": "output_text", "text": json.dumps(text)}]}]}


def intent():
    return {"observation_id": "obs-1", "decision": "select_candidate", "candidate_id": "c71",
            "reason_summary": "蓝色 B 位于左侧。", "evidence_regions": []}


def test_accepts_only_listed_candidate_and_matching_observation():
    assert parse_intent(response(intent()), "obs-1", {"c71", "c24"}) == intent()
    with pytest.raises(ProbeError, match="candidate"):
        parse_intent(response({**intent(), "candidate_id": "c99"}), "obs-1", {"c71"})
    with pytest.raises(ProbeError, match="observation"):
        parse_intent(response(intent()), "older", {"c71"})


def test_incomplete_and_refusal_are_not_decisions():
    with pytest.raises(ProbeError, match="incomplete"):
        parse_intent(response(intent(), "incomplete"), "obs-1", {"c71"})
    value = {"status": "completed", "output": [{"type": "message", "role": "assistant",
                                                "content": [{"type": "refusal"}]}]}
    with pytest.raises(ProbeError, match="refusal"):
        parse_intent(value, "obs-1", {"c71"})


def test_validates_evidence_and_nonselect_contract():
    with pytest.raises(ProbeError, match="bounds"):
        parse_intent(response({**intent(), "evidence_regions": [
            {"bbox_xyxy_norm": [0.2, 0.3, 0.1, 0.5], "label": "sign"}]}), "obs-1", {"c71"})
    with pytest.raises(ProbeError, match="candidate"):
        parse_intent(response({**intent(), "decision": "hold"}), "obs-1", {"c71"})


def test_diagnostic_mount_rotates_optical_forward_about_base_vertical():
    from navigation.camera_mount import OPTICAL_XYZW, yaw_mount_xyzw

    assert yaw_mount_xyzw(OPTICAL_XYZW, 0) == OPTICAL_XYZW
    x, y, z, w = yaw_mount_xyzw(OPTICAL_XYZW, 90)
    forward = (2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x*x + y*y))
    assert forward == pytest.approx((0.0, 1.0, 0.0))
