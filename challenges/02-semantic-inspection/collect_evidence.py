"""Export an explicit evidence whitelist; never copy private Agent homes or ledgers."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import zipfile


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--attempts", type=Path, nargs="+", required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError("Refuse to overwrite evidence export")
    a.output.mkdir(parents=True)
    ledger = []
    for attempt in a.attempts:
        dst = a.output / attempt.name
        dst.mkdir()
        for name in (
            "start.json",
            "result.json",
            "acceptance.json",
            "diagnosis.json",
            "recording-acceptance.json",
            "recording-diagnosis.json",
        ):
            if (attempt / name).is_file():
                shutil.copyfile(attempt / name, dst / name)
        root = attempt / "native"
        target = dst / "native"
        target.mkdir()
        for name in (
            "execution_config.json",
            "body.json",
            "body-effective.json",
            "semantic-inventory.json",
            "source-freeze.json",
            "canonical-receipts.json",
            "canonical-receipts-final.json",
            "task-kernel.json",
            "sdk-usage.json",
            "usage.json",
        ):
            if (root / name).is_file():
                shutil.copyfile(root / name, target / name)
        for name in (
            "actions",
            "semantic-evidence",
            "frozen-source",
            "frozen-semantic-source",
        ):
            if (root / name).is_dir():
                for item in (root / name).rglob("*"):
                    if item.is_symlink():
                        raise ValueError("Symlink in evidence whitelist")
                    if (
                        item.is_file()
                        and "__pycache__" not in item.parts
                        and item.suffix != ".pyc"
                    ):
                        path = target / item.relative_to(root)
                        path.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copyfile(item, path)
        # Export tool names/timestamps only; omit thinking, prose, arbitrary shell args and auth.
        events = []
        for session in (root / "home/agent/sessions").glob("*.jsonl"):
            for line in session.read_text().splitlines():
                row = json.loads(line)
                msg = row.get("message", {})
                if msg.get("role") != "assistant":
                    continue
                for content in msg.get("content", []):
                    if content.get("type") == "toolCall":
                        events.append(
                            {
                                "wall_time": datetime.fromisoformat(
                                    row["timestamp"].replace("Z", "+00:00")
                                ).timestamp(),
                                "name": content["name"],
                            }
                        )
        (dst / "visible-tool-events.json").write_text(
            json.dumps(events, indent=2) + "\n"
        )
        result = json.loads((attempt / "result.json").read_text())
        acceptance = json.loads((attempt / "acceptance.json").read_text())
        freeze = json.loads((root / "source-freeze.json").read_text())
        recording_status = result.get("recording_status", "NOT_REQUESTED")
        if (attempt / "recording-diagnosis.json").is_file():
            recording_status = json.loads(
                (attempt / "recording-diagnosis.json").read_text()
            )["status"]
        ledger.append(
            {
                "attempt": attempt.name,
                "physical_status": acceptance["status"],
                "recording_status": recording_status,
                "lab_sha": freeze["git_sha"],
                "upstream_sha": freeze["rosclaw_upstream"]["git_sha"],
                "development_only": True,
                "manual_interventions": result["manual_interventions"],
            }
        )
    (a.output / "attempt-ledger.json").write_text(json.dumps(ledger, indent=2) + "\n")
    manifest = {
        str(p.relative_to(a.output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(a.output.rglob("*"))
        if p.is_file()
    }
    (a.output / "manifest-sha256.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    archive = a.output.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for f in sorted(a.output.rglob("*")):
            if f.is_file():
                z.write(f, f.relative_to(a.output))
    print(
        json.dumps(
            {
                "archive": str(archive),
                "bytes": archive.stat().st_size,
                "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            }
        )
    )


if __name__ == "__main__":
    main()
