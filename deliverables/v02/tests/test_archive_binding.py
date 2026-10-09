"""Metadata/hash gates only; these fixtures do not claim physical acceptance."""

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location(
    "release_verify", Path(__file__).parents[1] / "verify_evidence.py"
)
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def native(*, changed_config=False, body="body", extra=False):
    config = json.dumps({"body_id": "sim", "body_snapshot_hash": "body"}).encode()
    freeze = {
        "git_sha": "runtime",
        "working_tree_dirty": False,
        "rosclaw_upstream": {"git_sha": "upstream", "working_tree_dirty": False},
        "docker_image": "image",
        "execution_config_sha256": hashlib.sha256(config).hexdigest(),
        "source_hashes": {"executor.py": hashlib.sha256(b"original").hexdigest()},
    }
    files = {
        "source-freeze.json": json.dumps(freeze).encode(),
        "execution_config.json": config if not changed_config else config + b" ",
        "body.json": json.dumps(
            {"body_id": "sim", "effective_body_hash": body}
        ).encode(),
        "frozen-source/executor.py": b"original",
    }
    manifest = {k: hashlib.sha256(v).hexdigest() for k, v in files.items()}
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as z:
        for k, v in files.items():
            z.writestr(k, v)
        z.writestr("public-evidence-manifest.json", json.dumps(manifest))
        if extra:
            z.writestr("unlisted.json", "{}")
    buffer.seek(0)
    return zipfile.ZipFile(buffer)


def check(z, source=None):
    return verify.check_native_binding(
        z,
        source=source,
        runtime="runtime",
        upstream="upstream",
        body="body",
        image="image",
    )


class BindingTests(unittest.TestCase):
    def test_correct_declared_binding(self):
        with native() as z:
            config, _ = check(z)
        self.assertEqual(config["body_snapshot_hash"], "body")

    def test_self_consistent_manifest_cannot_hide_changed_frozen_configuration(self):
        with (
            native(changed_config=True) as z,
            self.assertRaisesRegex(ValueError, "configuration"),
        ):
            check(z)

    def test_body_descriptor_must_agree(self):
        with (
            native(body="different") as z,
            self.assertRaisesRegex(ValueError, "Body snapshot"),
        ):
            check(z)

    def test_unlisted_members_are_rejected(self):
        with (
            native(extra=True) as z,
            self.assertRaisesRegex(ValueError, "exact archive membership"),
        ):
            check(z)

    def test_changed_checkout_cannot_be_reported_as_frozen_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "executor.py").write_bytes(b"changed")
            with (
                native() as z,
                self.assertRaisesRegex(ValueError, "Checked-out runtime file differs"),
            ):
                check(z, root)


if __name__ == "__main__":
    unittest.main()
