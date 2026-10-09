"""Verify a public archive and replay each successful attempt using its frozen code."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import zipfile


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--rosclaw-source", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError("Existing output refused")
    root = a.output.resolve()
    root.mkdir(parents=True)
    with zipfile.ZipFile(a.archive) as archive:
        for entry in archive.infolist():
            path = (root / entry.filename).resolve()
            if (
                not path.is_relative_to(root)
                or entry.external_attr >> 16 & 0o170000 == 0o120000
            ):
                raise ValueError("Archive contains escaping path or symlink")
        archive.extractall(root)
    manifest = json.loads((root / "manifest-sha256.json").read_text())
    for name, expected in manifest.items():
        path = (root / name).resolve()
        if (
            not path.is_relative_to(root)
            or hashlib.sha256(path.read_bytes()).hexdigest() != expected
        ):
            raise ValueError("Manifest mismatch: " + name)
    source = a.rosclaw_source.resolve()
    source_sha = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    results = []
    for attempt in json.loads((root / "attempt-ledger.json").read_text()):
        name = attempt["attempt"]
        if attempt["physical_status"] != "PASS":
            results.append(
                {
                    "attempt": name,
                    "original_status": attempt["physical_status"],
                    "replay_status": "NOT_REPLAYED",
                    "reason": "Retained incomplete original mission",
                }
            )
            continue
        if source_sha != attempt["upstream_sha"]:
            raise ValueError(
                "Checkout must match successful attempt upstream SHA: "
                + attempt["upstream_sha"]
            )
        native = root / name / "native"
        env = dict(
            os.environ,
            PYTHONPATH=os.pathsep.join(
                map(
                    str,
                    [
                        source / "src",
                        native / "frozen-source/evaluator",
                        native / "frozen-source/rosclaw",
                    ],
                )
            ),
        )
        replay = root / (name + "-replay.json")
        result = subprocess.run(
            [
                str(source / ".venv/bin/python"),
                str(native / "frozen-semantic-source/evaluate.py"),
                "--directory",
                str(native),
                "--output",
                str(replay),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=90,
        )
        proof = json.loads(replay.read_text())
        results.append(
            {
                "attempt": name,
                "original_status": "PASS",
                "replay_status": proof["status"],
                "exit_code": result.returncode,
            }
        )
    status = (
        "PASS"
        if all(r["replay_status"] in ("PASS", "NOT_REPLAYED") for r in results)
        else "FAIL"
    )
    record = {
        "status": status,
        "scope": "archive consistency only; not a new simulation or independent engineer run",
        "attempts": results,
    }
    (root / "offline-replay.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))
    raise SystemExit(0 if status == "PASS" else 1)


if __name__ == "__main__":
    main()
