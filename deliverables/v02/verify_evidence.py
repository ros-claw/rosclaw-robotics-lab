#!/usr/bin/env python3
"""Offline release consistency checks; never new physical or external acceptance."""

import argparse
from collections import Counter
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import sys
import tempfile
import zipfile

LAB = Path(__file__).resolve().parents[2]
CHALLENGE = LAB / "challenges/01-isaac-warehouse-patrol"
sys.path.insert(0, str(CHALLENGE / "evaluator"))
from replay_archive import replay  # noqa: E402

RUNTIME = "09739cb34c8a1d37ad95cfa4f146ad1e47798ec3"
UPSTREAM = "21838614bb14c39599b8acef731b2b64dad3b92a"
BODY = "7665b05487c9f95de7b4f195f7f7b59da10f22a1285b4e13fba1437cb7988fdf"


def checked_members(archive, manifest_name):
    names = archive.namelist()
    if len(names) != len(set(names)):
        raise ValueError("Duplicate archive member")
    for name in names:
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("Unsafe archive path")
    manifest = json.loads(archive.read(manifest_name))
    if set(names) != set(manifest) | {manifest_name}:
        raise ValueError("Manifest does not cover exact archive membership")
    for name, expected in manifest.items():
        if hashlib.sha256(archive.read(name)).hexdigest() != expected:
            raise ValueError("Public file hash mismatch: " + name)
    return manifest


def check_native_binding(archive, *, source, runtime, upstream, body, image):
    checked_members(archive, "public-evidence-manifest.json")
    freeze = json.loads(archive.read("source-freeze.json"))
    config_bytes = archive.read("execution_config.json")
    config = json.loads(config_bytes)
    declared_body = json.loads(archive.read("body.json"))
    if hashlib.sha256(config_bytes).hexdigest() != freeze["execution_config_sha256"]:
        raise ValueError("Execution configuration does not match frozen hash")
    if freeze["git_sha"] != runtime or freeze["working_tree_dirty"]:
        raise ValueError("Unexpected or dirty runtime source")
    upstream_info = freeze["rosclaw_upstream"]
    if upstream_info["git_sha"] != upstream or upstream_info["working_tree_dirty"]:
        raise ValueError("Unexpected or dirty upstream source")
    if freeze["docker_image"] != image:
        raise ValueError("Unexpected local Docker image ID")
    if (
        config["body_snapshot_hash"] != body
        or declared_body["effective_body_hash"] != body
    ):
        raise ValueError("Body snapshot declaration mismatch")
    if declared_body["body_id"] != config["body_id"]:
        raise ValueError("Body ID mismatch")
    for name, expected in freeze["source_hashes"].items():
        relative = PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Unsafe frozen source path")
        if (
            hashlib.sha256(archive.read("frozen-source/" + name)).hexdigest()
            != expected
        ):
            raise ValueError("Frozen source mismatch: " + name)
        if (
            source is not None
            and hashlib.sha256((source / name).read_bytes()).hexdigest() != expected
        ):
            raise ValueError("Checked-out runtime file differs: " + name)
    return config, freeze["source_hashes"]


def check_plan(rows, amendment, original):
    expected = {
        f"{i:02d}-{c}"
        for i in range(1, 6)
        for c in ["standard", "reordered", "unmapped-box"]
    }
    expected.add("06-standard")
    if len(rows) != 16 or {r["attempt"] for r in rows} != expected:
        raise ValueError(
            "Expected original fifteen attempts plus predeclared standard appendix"
        )
    if Counter(r["condition"] for r in rows) != {
        "standard": 6,
        "reordered": 5,
        "unmapped-box": 5,
    }:
        raise ValueError("Unexpected condition counts")
    extra = next(r for r in rows if r["attempt"] == "06-standard")
    if amendment["declared_wall_time"] >= extra["started_wall_time"]:
        raise ValueError("Appendix was not declared before execution")
    if original["attempts"] != 15 or original["passes"] != sum(
        r["status"] == "PASS" for r in rows if r["attempt"] != "06-standard"
    ):
        raise ValueError("Original fifteen-attempt outcome was changed")
    failures = [
        r["attempt"]
        for r in rows
        if r["attempt"] != "06-standard" and r["status"] != "PASS"
    ]
    if original["failures_and_incomplete"] != failures:
        raise ValueError("Original failed attempts were omitted or changed")
    for row in rows:
        if row["attempt"] != f"{row['repetition']:02d}-{row['condition']}":
            raise ValueError("Condition/repetition does not match attempt identity")


def register_reset(archive, observed):
    identities = {
        json.loads(line)["observer_id"]
        for line in archive.read("independent/physics-trajectory.jsonl").splitlines()
    }
    if len(identities) != 1:
        raise ValueError("Missing or mixed independent trajectory observer identity")
    identity = next(iter(identities))
    if not identity or identity in observed:
        raise ValueError("Repeated independent reset observer identity")
    observed.add(identity)
    return identity


def verify(path, *, source=CHALLENGE, runtime=RUNTIME, upstream=UPSTREAM, body=BODY):
    reports = []
    with zipfile.ZipFile(path) as outer:
        checked_members(outer, "public-file-manifest.json")
        freeze = json.loads(outer.read("final-native/freeze.json"))
        rows = [
            json.loads(line)
            for line in outer.read("final-native/attempts.jsonl").splitlines()
        ]
        amendment = json.loads(outer.read("final-native/plan-amendment.json"))
        original = json.loads(outer.read("final-native/original-fifteen-summary.json"))
        check_plan(rows, amendment, original)
        supervisor = outer.read(
            "release-documents/evidence-scripts/standard-appendix-supervisor.py"
        )
        if hashlib.sha256(supervisor).hexdigest() != amendment["supervisor_sha256"]:
            raise ValueError("Appendix supervisor does not match predeclared source")
        if freeze["lab_commit"] != runtime or freeze["rosclaw_commit"] != upstream:
            raise ValueError("Outer source freeze mismatch")
        observers = set()
        all_reset_observers = set()
        hashes = None
        native_counts = Counter()
        for row in rows:
            if row["source_commit"] != runtime or row["manual_interventions"]:
                raise ValueError("Changed/intervened attempt: " + row["attempt"])
            recorded = json.loads(
                outer.read("final-native/" + row["attempt"] + "/result.json")
            )
            if recorded != row:
                raise ValueError("Attempt result does not match complete ledger")
            if row["status"] != "PASS":
                if row["status"] not in {"FAIL", "INCOMPLETE", "INTERRUPTED"}:
                    raise ValueError("Unfinished attempt in release")
                failed_archive = "final-native/" + row["attempt"] + ".zip"
                if failed_archive in outer.namelist():
                    with zipfile.ZipFile(
                        io.BytesIO(outer.read(failed_archive))
                    ) as inner:
                        _, current_hashes = check_native_binding(
                            inner,
                            source=source,
                            runtime=runtime,
                            upstream=upstream,
                            body=body,
                            image=freeze["docker_image_id"],
                        )
                        if hashes is not None and current_hashes != hashes:
                            raise ValueError(
                                "Failed attempt source differs from frozen batch"
                            )
                        hashes = current_hashes
                        register_reset(inner, all_reset_observers)
                    native_counts[row["condition"]] += 1
                    acceptance = json.loads(
                        outer.read(
                            "final-native/" + row["attempt"] + "/acceptance.json"
                        )
                    )
                    if acceptance["status"] == "PASS":
                        raise ValueError("Failed Native attempt mislabeled as accepted")
                reports.append(
                    {"attempt": row["attempt"], "status": row["status"], "visits": 0}
                )
                continue
            native_counts[row["condition"]] += 1
            payload = outer.read("final-native/" + row["attempt"] + ".zip")
            with zipfile.ZipFile(io.BytesIO(payload)) as inner:
                config, current_hashes = check_native_binding(
                    inner,
                    source=source,
                    runtime=runtime,
                    upstream=upstream,
                    body=body,
                    image=freeze["docker_image_id"],
                )
                if hashes is not None and current_hashes != hashes:
                    raise ValueError("Runtime source hashes differ between attempts")
                hashes = current_hashes
                reset_identity = register_reset(inner, all_reset_observers)
                wanted_order = (
                    ["entry", "shelf", "aisle", "home"]
                    if row["condition"] == "standard"
                    else ["aisle", "entry", "shelf", "home"]
                )
                if config["expected_order"] != wanted_order or bool(
                    config.get("require_obstacle_evidence")
                ) != (row["condition"] == "unmapped-box"):
                    raise ValueError("Condition/order/obstacle contract mismatch")
                final = json.loads(inner.read("actions/patrol.verification.json"))
                actual_observers = set()
                for visit in final["visits"]:
                    evidence = json.loads(
                        inner.read(
                            "actions/" + PurePosixPath(visit["artifact"]["path"]).name
                        )
                    )
                    actual_observers.update(
                        s.get("observer_id") for s in evidence["trajectory"]
                    )
            with tempfile.TemporaryDirectory(
                prefix="rosclaw-offline-replay-"
            ) as directory:
                native = Path(directory) / "native.zip"
                native.write_bytes(payload)
                proof = replay(native)
            if proof["status"] != "PASS":
                raise ValueError("Physical receipt replay failed: " + row["attempt"])
            acceptance = json.loads(
                outer.read("final-native/" + row["attempt"] + "/acceptance.json")
            )
            ids = acceptance["observer_ids"]
            if acceptance["status"] != "PASS" or set(ids) != actual_observers:
                raise ValueError("Acceptance does not match replayed reset evidence")
            if len(ids) != 1 or ids[0] in observers or ids[0] != reset_identity:
                raise ValueError(
                    "Missing or repeated independent reset observer identity"
                )
            observers.add(ids[0])
            reports.append(
                {
                    "attempt": row["attempt"],
                    "status": proof["status"],
                    "visits": proof["visits"],
                }
            )
        if any(native_counts[c] < 5 for c in ["standard", "reordered", "unmapped-box"]):
            raise ValueError("Fewer than five completed Native attempts in a condition")
        passed_counts = Counter(r["condition"] for r in rows if r["status"] == "PASS")
        summary = json.loads(outer.read("final-native/summary.json"))
        if summary["passes"] != len(observers) or summary["attempts"] != len(rows):
            raise ValueError("Published summary counts do not match replayed ledger")
        if summary["accepted_per_condition"] != dict(passed_counts):
            raise ValueError("Published accepted condition counts do not match ledger")
    return {
        "status": "PASS",
        "scope_note": "PASS means offline consistency, not all mission attempts succeeded",
        "five_accepted_per_condition_goal_met": all(
            passed_counts[c] >= 5 for c in ["standard", "reordered", "unmapped-box"]
        ),
        "completed_native_attempts_per_condition": dict(native_counts),
        "attempts": len(reports),
        "physically_accepted_attempts": len(observers),
        "failure_records_retained": len(reports) - len(observers),
        "unique_reset_observers": len(observers),
        "unique_native_reset_observers_including_failed": len(all_reset_observers),
        "scope": "offline source/config/Body-declaration/receipt/physical evidence consistency; not live reproduction or signature attestation",
        "reports": reports,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    p.add_argument(
        "--runtime-source",
        type=Path,
        default=CHALLENGE,
        help="actual checked-out challenge directory; code files must match frozen hashes",
    )
    a = p.parse_args()
    result = verify(a.archive, source=a.runtime_source)
    print(json.dumps(result, indent=2))
