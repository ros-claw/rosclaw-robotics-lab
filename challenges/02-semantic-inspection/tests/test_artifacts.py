import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifacts import artifact_path


class ArtifactScopeTests(unittest.TestCase):
    def test_relocation_keeps_original_recorded_bytes_and_hash(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            (root / "actions").mkdir()
            p = root / "actions/proof.json"
            p.write_bytes(b'{"x":1}')
            ref = {
                "path": "/original/private-run/actions/proof.json",
                "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            }
            actual = artifact_path(
                root,
                {"execution_root": "/original/private-run"},
                ref,
                directory="actions",
            )
            self.assertEqual(actual, p)
            for bad in [
                "/unrelated/actions/proof.json",
                "/original/private-run/actions/../proof.json",
                "/original/private-run/actions/sub/proof.json",
            ]:
                with self.assertRaises(ValueError):
                    artifact_path(
                        root,
                        {"execution_root": "/original/private-run"},
                        {**ref, "path": bad},
                        directory="actions",
                    )
            p.write_text("changed")
            with self.assertRaises(ValueError):
                artifact_path(
                    root,
                    {"execution_root": "/original/private-run"},
                    ref,
                    directory="actions",
                )


if __name__ == "__main__":
    unittest.main()
