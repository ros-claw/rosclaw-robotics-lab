"""Independent public summary; no success is inferred from model prose."""

import argparse
import hashlib
import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path
from patrol import verify_visit
from obstacle import verify_obstacle


def report(root, target):
    config = json.loads((root / "execution_config.json").read_text())
    failures = []
    final_path = root / "actions/patrol.verification.json"
    if not final_path.exists():
        result = {
            "run_id": root.name,
            "status": "FAIL",
            "failure": "verified patrol artifact missing",
            "task": config["task"],
            "body_snapshot_hash": config["body_snapshot_hash"],
            "ground_truth_source": config["physics_directory"],
            "partial_visits": [
                {
                    "action_id": data.get("action_id"),
                    "site_id": data.get("site_id"),
                    "status": data.get("status", data.get("verification", {}).get("verification_status")),
                    "error": data.get("error"),
                    "nav2": data.get("nav2"),
                    "verification": data.get("verification"),
                    "artifact_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "sample_count": len(data.get("trajectory", [])),
                }
                for path in sorted((root / "actions").glob("*.json"))
                for data in [json.loads(path.read_text())]
            ],
            "memory_success_claimed": False,
        }
        target.write_text(json.dumps(result, indent=2) + "\n")
        return result
    final = json.loads(final_path.read_text())
    receipts = json.loads((root / "canonical-receipts-final.json").read_text())
    nav_receipts = {
        x["receipt"]["action_id"]: x["receipt"]
        for x in receipts
        if x.get("receipt")
        and x["receipt"].get("capability_id") == "navigation.navigate_to_pose"
    }
    memory = [
        x["receipt"]
        for x in receipts
        if x.get("receipt")
        and x["receipt"].get("capability_id") == "patrol.verify_and_remember"
        and x["receipt"].get("final_state") == "COMPLETED"
    ]
    visits, trajectory = [], []
    for visit in final["visits"]:
        artifact = visit["artifact"]
        path = Path(artifact["path"]).resolve()
        if (
            path.parent != (root / "actions").resolve()
            or hashlib.sha256(path.read_bytes()).hexdigest() != artifact["sha256"]
        ):
            failures.append("artifact path/hash mismatch")
            continue
        data = json.loads(path.read_text())
        receipt = nav_receipts.get(data["action_id"], {})
        if (
            receipt.get("body_id") != config["body_id"]
            or receipt.get("body_snapshot_hash") != config["body_snapshot_hash"]
            or receipt.get("final_state") != "COMPLETED"
            or receipt.get("evidence_domain") != "SIMULATION"
            or receipt.get("verification_result", {}).get("evidence_artifact")
            != artifact
        ):
            failures.append("canonical visit receipt binding failed")
        proof = verify_visit(
            data["trajectory"],
            config["sites"][data["site_id"]],
            data["nav2"],
            data["verification"]["sensor"],
            body_path=config["physics_body_path"],
        )
        if not proof["success"]:
            failures.append("independent visit verification failed")
        recoveries = max(
            (
                e["values"].get("number_of_recoveries", 0)
                for e in data["events"]
                if e["event"] == "feedback"
            ),
            default=0,
        )
        trajectory.extend(data["trajectory"])
        visits.append(
            {
                "site_id": data["site_id"],
                "action_id": data["action_id"],
                "receipt_id": receipt.get("receipt_id"),
                "nav2_status": data["nav2"]["status"],
                "position_xy": data["trajectory"][-1]["physics_transforms_xyzw"][0][:2],
                "recoveries": recoveries,
                "started_wall_time": data["started_wall_time"],
                "finished_wall_time": data["finished_wall_time"],
                **proof,
            }
        )
    obstacle_proof = None
    if config.get("require_obstacle_evidence") and visits:
        obstacle_proof = verify_obstacle(config["physics_directory"], trajectory,
                                        visits[0]["started_wall_time"], visits[-1]["finished_wall_time"],
                                        config.get("obstacle_static_map_validation"))
        if obstacle_proof["status"] != "PASS":
            failures.append("unmapped obstacle independent verification failed")
    if [v["site_id"] for v in visits] != config["expected_order"]:
        failures.append("operator task order mismatch")
    if not memory or final.get("memory_outcome") != "success":
        failures.append("verified existing Memory receipt missing")
    db = sqlite3.connect(f"file:{root / 'home/agentd/missions.db'}?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    task = db.execute(
        "select task_id,state,root_goal,updated_at from tasks order by created_at desc limit 1"
    ).fetchone()
    task = dict(task) if task else {}
    if task.get("state") != "SUCCEEDED":
        failures.append("TaskKernel is not SUCCEEDED")
    elif memory and datetime.fromisoformat(task["updated_at"]) < datetime.fromisoformat(
        memory[-1]["finished_at"]
    ):
        failures.append("TaskKernel completed before the full mission Memory receipt")
    db.close()
    sessions = list((root / "home/agent/sessions").rglob("*.jsonl"))
    messages = [
        json.loads(line).get("message", {})
        for p in sessions
        for line in p.read_text().splitlines()
    ]
    assistants = [m for m in messages if m.get("role") == "assistant"]
    calls = [
        {"name": b["name"], "arguments": b.get("arguments")}
        for m in assistants
        for b in m.get("content", [])
        if isinstance(b, dict) and b.get("type") == "toolCall"
    ]
    final_text = [
        b.get("text", "")
        for m in assistants
        if m.get("stopReason") == "stop"
        for b in m.get("content", [])
        if isinstance(b, dict) and b.get("type") == "text"
    ]
    user_times = [
        datetime.fromisoformat(json.loads(line)["timestamp"]).timestamp()
        for p in sessions
        for line in p.read_text().splitlines()
        if json.loads(line).get("message", {}).get("role") == "user"
    ]
    input_time = min(user_times) if user_times else None
    first_motion = next(
        (
            s["wall_time"]
            for s in trajectory
            if math.hypot(*s["linear_velocity_xyz"][:2]) > 0.05
        ),
        None,
    )
    distance = sum(
        math.dist(
            a["physics_transforms_xyzw"][0][:2], b["physics_transforms_xyzw"][0][:2]
        )
        for a, b in zip(trajectory, trajectory[1:])
    )
    result = {
        "schema_version": "rosclaw.isaac_native_run.v1",
        "run_id": root.name,
        "status": "FAIL" if failures else "PASS",
        "failures": failures,
        "task": config["task"],
        "task_kernel": task,
        "body_id": config["body_id"],
        "body_snapshot_hash": config["body_snapshot_hash"],
        "evidence_domain": "SIMULATION",
        "hardware_verified": False,
        "model_turns": len(assistants),
        "models": sorted(
            {m.get("provider", "") + "/" + m.get("model", "") for m in assistants}
        ),
        "tool_call_count": len(calls),
        "tool_calls": calls,
        "agent_final_text": final_text,
        "visits": visits,
        "obstacle_verification": obstacle_proof,
        "memory_id": final.get("memory_id"),
        "memory_outcome": final.get("memory_outcome"),
        "practice_files": [
            str(p.relative_to(root))
            for p in (root / "practice").rglob("*")
            if p.is_file()
        ],
        "ground_truth_source": config["physics_directory"],
        "observer_ids": sorted({s.get("observer_id", "legacy") for s in trajectory}),
        "trajectory_sample_count": len(trajectory),
        "actual_distance_m": distance,
        "physical_action_wall_seconds": sum(
            v["finished_wall_time"] - v["started_wall_time"] for v in visits
        ),
        "sim_seconds": trajectory[-1]["sim_time"] - trajectory[0]["sim_time"]
        if trajectory
        else None,
        "input_to_first_motion_wall_seconds": first_motion - input_time
        if first_motion and input_time
        else None,
        "observed_sim_to_wall_ratio": (trajectory[-1]["sim_time"] - trajectory[0]["sim_time"])
        / (trajectory[-1]["wall_time"] - trajectory[0]["wall_time"]) if len(trajectory)>1 else None,
        "startup_wall_seconds": (
            json.loads((Path(config["physics_directory"])/"run-ready.json").read_text())["wall_time"]
            - json.loads((Path(config["physics_directory"])/"run-start.json").read_text())["wall_time"]
        ) if (Path(config["physics_directory"])/"run-ready.json").exists() else None,
        "source_freeze": json.loads((root / "source-freeze.json").read_text()),
        "notes": [
            "Distance includes sampled visits and intervening position deltas.",
            "Raw model thinking and credentials are not exported.",
        ],
    }
    target.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    r = report(a.directory.resolve(), a.output)
    print(
        json.dumps(
            {
                "run_id": r["run_id"],
                "status": r["status"],
                "failures": r.get("failures"),
            }
        )
    )
