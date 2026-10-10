#!/usr/bin/env python3
"""Independent replay of physical feedback, canonical receipts and semantic task."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PATROL = HERE.parent / "01-isaac-warehouse-patrol"
for p in [PATROL / "evaluator", PATROL / "rosclaw", HERE]:
    sys.path.insert(0, str(p))
from artifacts import artifact_path
from rosclaw.body.schema import EffectiveBody
from patrol import verify_visit
from semantic_executor import SemanticMemoryExecutor


def evaluate(root, checker_type=SemanticMemoryExecutor):
    config = json.loads((root / "execution_config.json").read_text())
    failures = []
    visits = []
    checker = checker_type(None, root, config, None)
    body = config["body_snapshot_hash"]
    effective = EffectiveBody.from_dict(
        json.loads((root / "body-effective.json").read_text())
    )
    if (
        effective.compute_hash() != body
        or effective.safety["navigation_contract"] != config["semantic_contract"]
    ):
        raise ValueError("Effective Body or semantic contract binding mismatch")
    receipts = json.loads((root / "canonical-receipts-final.json").read_text())
    complete = [
        r["receipt"]
        for r in receipts
        if (r.get("receipt") or {}).get("final_state") == "COMPLETED"
    ]
    nav = [r for r in complete if r["capability_id"] == "navigation.navigate_to_pose"]
    nav.sort(key=lambda r: r["started_at"])
    for r in nav:
        if (
            r["body_snapshot_hash"] != body
            or r["body_id"] != config["body_id"]
            or r["mode"] != "SIMULATION"
            or r["evidence_domain"] != "SIMULATION"
            or r["evidence_level"] != "TASK_VERIFIED"
        ):
            raise ValueError("Canonical receipt Body/domain/evidence mismatch")
        artifact = r["verification_result"]["evidence_artifact"]
        p = artifact_path(root, config, artifact, directory="actions")
        data = json.loads(p.read_text())
        site = checker.target_for_visit(data)
        proof = verify_visit(
            data["trajectory"],
            site,
            data["nav2"],
            data["verification"]["sensor"],
            body_path=config["physics_body_path"],
        )
        checker.check_extra_evidence(data, proof)
        if not proof["success"] or data["action_id"] != r["action_id"]:
            raise ValueError("Independent physical visit replay failed")
        visits.append(
            {
                "site_id": data["site_id"],
                "started_wall_time": data["started_wall_time"],
                "finished_wall_time": data["finished_wall_time"],
                "verification": proof,
                "artifact": artifact,
                "target_prim": data["verification"]["semantic"]["proposal"][
                    "candidate"
                ]["target_prim"],
            }
        )
    if config.get("scenario") == "loading_inspection":
        final = json.loads((root / "actions/mission.verification.json").read_text())
        checker.agent_report = final["inspection_report"]
    checker.check_mission(visits)
    for a, b in zip(visits, visits[1:]):
        if a["finished_wall_time"] > b["started_wall_time"]:
            raise ValueError("Overlapping/reordered visits")
    memory = [
        r for r in complete if r["capability_id"] == "mission.verify_and_remember"
    ]
    if (
        len(memory) != 1
        or memory[0]["body_snapshot_hash"] != body
        or memory[0]["verification_result"].get("memory_outcome") != "success"
    ):
        failures.append("Missing complete canonical Memory receipt")
    verification = json.loads((root / "actions/mission.verification.json").read_text())
    if verification["verification_status"] != "PASS" or [
        r["action_id"] for r in nav
    ] != [r["action_id"] for r in verification["execution_receipts"]]:
        failures.append("Final ordered mission evidence mismatch")
    task = json.loads((root / "task-kernel.json").read_text())
    if task["state"] != "SUCCEEDED" or (
        memory and task["updated_at"] < memory[0]["finished_at"]
    ):
        failures.append("TaskKernel did not finish after verified Memory")
    freeze = json.loads((root / "source-freeze.json").read_text())
    if (
        freeze["working_tree_dirty"]
        or freeze["rosclaw_upstream"]["working_tree_dirty"]
        or freeze["native_build_stamp"]["commit"]
        != freeze["rosclaw_upstream"]["git_sha"]
    ):
        failures.append("Source/compiled Native pins not clean or consistent")
    if (
        hashlib.sha256((root / "execution_config.json").read_bytes()).hexdigest()
        != freeze["execution_config_sha256"]
    ):
        failures.append("Execution configuration differs from frozen hash")
    for f, h in freeze["source_hashes"].items():
        if hashlib.sha256((root / "frozen-source" / f).read_bytes()).hexdigest() != h:
            failures.append("Frozen runtime code/config hash mismatch")
    for f, h in freeze["semantic_source_hashes"].items():
        if (
            hashlib.sha256(
                (root / "frozen-semantic-source" / f).read_bytes()
            ).hexdigest()
            != h
        ):
            failures.append("Frozen semantic code hash mismatch")
    return {
        "schema_version": "rosclaw.semantic_acceptance.v1",
        "status": "PASS" if not failures else "FAIL",
        "evidence_domain": "SIMULATION",
        "hardware_verified": False,
        "source_commit": freeze["git_sha"],
        "rosclaw_commit": freeze["rosclaw_upstream"]["git_sha"],
        "body_snapshot_hash": body,
        "observer_id": config["initial_physics"]["observer_id"],
        "task": config["task"],
        "selection_rule": config["selection_rule"],
        "require_target_lidar": config["require_target_lidar"],
        "inspection_scope": config["semantic_contract"]["inspection_scope"],
        "visits": visits,
        "memory_outcome": verification["memory_outcome"],
        "task_kernel_state": task["state"],
        "failures": failures,
        "holdout_evaluation": False,
        "manual_interventions": [],
        **({"loading_inspection": checker.replay_summary,
             "inspection_report": checker.agent_report,
             "observation_views": checker.replay_views}
            if config.get("scenario") == "loading_inspection" else {}),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    try:
        result = evaluate(a.directory.resolve())
    except Exception as exc:
        result = {
            "status": "FAIL",
            "error": str(exc),
            "evidence_domain": "SIMULATION",
            "acceptance_incomplete": True,
        }
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    raise SystemExit(0 if result["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
