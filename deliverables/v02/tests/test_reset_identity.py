"""Metadata counterexamples; not new physical simulator acceptance."""

import importlib.util
import io
import json
from pathlib import Path
import unittest
import zipfile

spec = importlib.util.spec_from_file_location(
    "reset_verify", Path(__file__).parents[1] / "verify_evidence.py"
)
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def archive(ids):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as z:
        z.writestr(
            "independent/physics-trajectory.jsonl",
            "".join(json.dumps({"observer_id": i}) + "\n" for i in ids),
        )
    buffer.seek(0)
    return zipfile.ZipFile(buffer)


class ResetTests(unittest.TestCase):
    def test_failed_and_successful_runs_cannot_reuse_reset_identity(self):
        observed = set()
        with archive(["same"]) as first:
            verify.register_reset(first, observed)
        with (
            archive(["same"]) as second,
            self.assertRaisesRegex(ValueError, "Repeated"),
        ):
            verify.register_reset(second, observed)

    def test_mixed_trajectory_observers_are_rejected(self):
        with (
            archive(["first", "second"]) as z,
            self.assertRaisesRegex(ValueError, "mixed"),
        ):
            verify.register_reset(z, set())


if __name__ == "__main__":
    unittest.main()
