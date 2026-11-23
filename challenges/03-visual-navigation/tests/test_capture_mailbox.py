from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.capture_mailbox import request_directory, eligible_frame


def test_service_rejects_traversal_invalid_deadlines_and_stale_camera_frames():
    root = Path("/run")
    valid = {"capture": "rgbd-2d3f4a5b", "requested_ns": 100, "deadline_ns": 200}
    assert request_directory(root, valid, 110) == root / "rgbd-2d3f4a5b"
    assert request_directory(root, {**valid, "capture": "../rgbd-2d3f4a5b"}, 110) is None
    assert request_directory(root, valid, 200) is None
    assert request_directory(root, valid, 99) is None
    assert not eligible_frame(99, 100, 250_000_000, 300_000_000)
    assert not eligible_frame(101, 100, 1, 300_000_000)
    assert eligible_frame(101, 100, 250_000_000, 300_000_000)


def test_request_waits_for_same_directory_completion(tmp_path):
    import json
    import threading
    import time
    from navigation.capture_mailbox import request_frame, write_atomic

    destination = tmp_path / "rgbd-12345678"

    def complete():
        request = tmp_path / "capture-request.json"
        while not request.exists():
            time.sleep(.005)
        data = json.loads(request.read_text())
        assert data["capture"] == destination.name
        destination.mkdir()
        write_atomic(destination / "observation_summary.json", {"captured": 1})

    worker = threading.Thread(target=complete, daemon=True)
    worker.start()
    assert request_frame(tmp_path, destination, .5) == {"captured": 1}
    worker.join(.2)
