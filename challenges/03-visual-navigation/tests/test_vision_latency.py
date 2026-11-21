from io import BytesIO
from pathlib import Path
import sys

from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.vision_profile import image_payload, exchange_stats


def test_lean_audit_keeps_image_coordinates_and_changes_only_transport_profile():
    buffer = BytesIO()
    Image.new("RGB", (960, 540), "#0076CB").save(buffer, format="PNG")
    image = buffer.getvalue()
    assert image_payload(image, "standard") == (image, "high")
    wire, detail = image_payload(image, "lean")
    assert Image.open(BytesIO(wire)).size == (640, 360)
    assert Image.open(BytesIO(wire)).convert("RGB").getpixel((320, 180)) == (0, 118, 203)
    assert detail == "low"


def test_exchange_stats_distinguish_setup_from_wait_to_response():
    data, stats = exchange_stats('{"id":"value"}\n200 0.010 0.070 0.110 0.120 1.700 1.710 3000')
    assert data == '{"id":"value"}'
    assert stats["http_status"] == 200 and stats["uploaded_bytes"] == 3000
    assert stats["connection_setup_seconds"] == 0.12
    assert stats["elapsed_after_connect_seconds"] == pytest.approx(1.59)


def test_compact_reply_requires_exact_schema_and_normalizes_for_local_audits():
    import json
    from navigation.luna_probe import AUDIT_DECISIONS, ProbeError, parse_intent

    def response(value):
        return {"status": "completed", "output": [{"type": "message", "role": "assistant",
                "content": [{"type": "output_text", "text": json.dumps(value)}]}]}

    reply = {"observation_id": "view", "decision": "review", "reason_summary": "橙色挡板"}
    result = parse_intent(response(reply), "view", {}, AUDIT_DECISIONS, compact=True)
    assert result == {**reply, "candidate_id": None, "evidence_regions": []}
    with pytest.raises(ProbeError):
        parse_intent(response({**reply, "candidate_id": "left"}), "view", {}, AUDIT_DECISIONS, compact=True)
