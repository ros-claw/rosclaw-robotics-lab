"""Verify canonical SIM visit evidence and persist through existing ROS Practice."""

import hashlib
import json
from pathlib import Path
from rosclaw.connectors.ros.intelligence.evidence import emit_expert_evidence
from rosclaw.kernel import (
    ActionExecutionResult,
    ActionState,
    EvidenceDomain,
    EvidenceLevel,
    ExecutionMode,
)
from rosclaw.runtime.event import RuntimeEvent
from patrol import verify_visit
from obstacle import verify_obstacle


class PatrolMemoryExecutor:
    def __init__(self, runtime, root, config, recorder_bus):
        self.runtime, self.root, self.config, self.bus = (
            runtime,
            Path(root),
            config,
            recorder_bus,
        )

    def __call__(self, action):
        try:
            if (
                action.execution_mode is not ExecutionMode.SIMULATION
                or action.body_id != self.config["body_id"]
                or action.body_snapshot_hash != self.config["body_snapshot_hash"]
                or set(action.arguments) != {"action_ids"}
            ):
                raise ValueError("SIM-only Body-bound memory action required")
            ids = action.arguments["action_ids"]
            if not ids or len(set(ids)) != len(ids):
                raise ValueError("unique canonical action IDs required")
            receipts, visits = [], []
            for action_id in ids:
                obj = self.runtime.action_gateway.get_receipt(action_id)
                r = obj.to_dict() if obj else {}
                if (
                    r.get("body_id") != action.body_id
                    or r.get("body_snapshot_hash") != action.body_snapshot_hash
                    or r.get("mode") != "SIMULATION"
                    or r.get("evidence_domain") != "SIMULATION"
                    or r.get("final_state") != "COMPLETED"
                    or r.get("capability_id") != "navigation.navigate_to_pose"
                    or r.get("evidence_level") != "TASK_VERIFIED"
                ):
                    raise ValueError("unverified or mismatched canonical receipt")
                artifact = r["verification_result"]["evidence_artifact"]
                path = Path(artifact["path"]).resolve()
                if (
                    path.parent != (self.root / "actions").resolve()
                    or hashlib.sha256(path.read_bytes()).hexdigest()
                    != artifact["sha256"]
                ):
                    raise ValueError("evidence artifact path/hash mismatch")
                data = json.loads(path.read_text())
                site_id = data["site_id"]
                proof = verify_visit(
                    data["trajectory"],
                    self.config["sites"][site_id],
                    data["nav2"],
                    data["verification"]["sensor"],
                    body_path=self.config["physics_body_path"],
                )
                if data["action_id"] != action_id or not proof["success"]:
                    raise ValueError("independent visit replay failed")
                if (
                    visits
                    and data["started_wall_time"] < visits[-1]["finished_wall_time"]
                ):
                    raise ValueError("overlapping or reordered visits")
                receipts.append(r)
                visits.append(
                    {
                        "site_id": site_id,
                        "started_wall_time": data["started_wall_time"],
                        "finished_wall_time": data["finished_wall_time"],
                        "verification": proof,
                        "artifact": artifact,
                    }
                )
            if [v["site_id"] for v in visits] != self.config["expected_order"]:
                raise ValueError(
                    "visits do not satisfy the operator task order/skipped sites"
                )
            obstacle_proof = None
            if self.config.get("require_obstacle_evidence"):
                trajectories = [sample for v in visits
                                for sample in json.loads(Path(v["artifact"]["path"]).read_text())["trajectory"]]
                obstacle_proof = verify_obstacle(self.config["physics_directory"], trajectories,
                                                visits[0]["started_wall_time"], visits[-1]["finished_wall_time"],
                                                self.config.get("obstacle_static_map_validation"))
                if obstacle_proof["status"] != "PASS":
                    raise ValueError("unmapped obstacle evidence failed: " + str(obstacle_proof["failures"]))
            verification = {
                "schema_version": "rosclaw.isaac_patrol_verification.v1",
                "mission_id": self.config["mission_id"],
                "body_id": action.body_id,
                "body_snapshot_hash": action.body_snapshot_hash,
                "task": self.config["task"],
                "verification_status": "PASS",
                "success": True,
                "evidence_domain": "SIMULATION",
                "hardware_verified": False,
                "usable_for_real_execution": False,
                "execution_receipts": receipts,
                "visits": visits,
                "obstacle_verification": obstacle_proof,
            }
            emit_expert_evidence(
                self.runtime.event_bus,
                "rosclaw.ros.verification.completed",
                {
                    "robot_id": action.body_id,
                    "mission_id": self.config["mission_id"],
                    "verification": verification,
                },
            )
            stored = (
                self.runtime.memory.get_experience(self.config["mission_id"])
                if self.runtime.memory
                else None
            )
            if not stored or stored.get("outcome") != "success":
                raise ValueError("existing Memory did not persist verified success")
            target = self.root / "actions" / "patrol.verification.json"
            verification["memory_id"] = self.config["mission_id"]
            verification["memory_outcome"] = stored["outcome"]
            target.write_text(json.dumps(verification, indent=2) + "\n")
            self.bus.publish(
                RuntimeEvent(
                    type="practice.stop",
                    source="runtime",
                    robot=action.body_id,
                    body_id=action.body_id,
                    payload={
                        "outcome": "SUCCESS",
                        "metadata": {
                            "verification_artifact": str(target),
                            "evidence_domain": "SIMULATION",
                        },
                    },
                )
            )
            return ActionExecutionResult(
                final_state=ActionState.COMPLETED,
                evidence_level=EvidenceLevel.TASK_VERIFIED,
                evidence_domain=EvidenceDomain.SIMULATION,
                policy_decision={"allowed": True},
                dispatch_result={"accepted": True},
                artifacts=[str(target)],
                verification_result={
                    "success": True,
                    "mission_verification_artifact": str(target),
                    "memory_id": self.config["mission_id"],
                    "memory_outcome": stored["outcome"],
                },
            )
        except Exception as exc:
            return ActionExecutionResult(
                final_state=ActionState.BLOCKED,
                evidence_level=EvidenceLevel.REQUESTED,
                evidence_domain=EvidenceDomain.SIMULATION,
                policy_decision={"allowed": False},
                errors=[{"code": "PATROL_MEMORY_REJECTED", "message": str(exc)}],
            )
