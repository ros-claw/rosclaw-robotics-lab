from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_old_evidence_discloses_model_only_at_live_preflight():
    from viewer.replay import available_times

    events = [{"event": "final_twist_source_verified", "host_mono_ns": "200"},
              {"event": "intent_preflight", "host_mono_ns": "300"},
              {"event": "odometry_stopped", "host_mono_ns": "400"}]
    assert available_times({}, events) == (200, 300)
    assert available_times({"request_host_mono_ns": "100", "received_host_mono_ns": "150"},
                           events) == (100, 150)
