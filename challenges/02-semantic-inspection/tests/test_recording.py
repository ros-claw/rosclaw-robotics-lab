import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from recording import verify_recording


class RecordingTests(unittest.TestCase):
    def test_missing_corrupt_or_nonfinite_index_reports_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertEqual(verify_recording(root, 10, 20)["status"], "FAIL")
            index = root / "timestamps.jsonl"
            index.write_text("incomplete JSON")
            self.assertEqual(verify_recording(root, 10, 20)["status"], "FAIL")
            index.write_text(json.dumps({"wall_time": float("nan")}))
            self.assertEqual(verify_recording(root, 10, 20)["status"], "FAIL")

    def test_sync_and_full_window_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows = []
            for index, stamp in enumerate((10, 15, 20)):
                views = {}
                for camera in ("follow", "robot", "top-close"):
                    name = f"{camera}-{index}.png"
                    (root / name).write_bytes(b"test fixture")
                    views[camera] = {"frame": name, "render_frame": index}
                rows.append({"wall_time": stamp, "views": views})
            log = root / "timestamps.jsonl"
            log.write_text("\n".join(map(json.dumps, rows)))
            self.assertEqual(verify_recording(root, 11, 19)["status"], "PASS")
            self.assertEqual(verify_recording(root, 11, 21)["status"], "FAIL")
            rows[1]["views"]["robot"]["render_frame"] = 99
            log.write_text("\n".join(map(json.dumps, rows)))
            self.assertEqual(verify_recording(root, 11, 19)["status"], "FAIL")

    def test_missing_file_is_not_a_recorded_view(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            views = {
                name: {"frame": name + ".png", "render_frame": 1}
                for name in ("follow", "robot", "top-close")
            }
            (root / "timestamps.jsonl").write_text(
                json.dumps({"wall_time": 10, "views": views})
            )
            self.assertEqual(verify_recording(root, 10, 10)["status"], "FAIL")
