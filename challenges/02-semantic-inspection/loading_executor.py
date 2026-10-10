"""Loading-area single-goal executor reusing immutable proposal, Nav2 and receipts."""

import json
import math
from pathlib import Path
import subprocess
import time
import yaml
import numpy as np
from evidence_io import write_json_atomic
from semantic_executor import (
    SemanticExecutor,
    SemanticMemoryExecutor,
    file_sha,
    checked_artifact,
)
from safety import canonical_hash, swept_footprint, angle_delta
from propose_observation import free_grid
from loading_geometry import (
    entities,
    relations,
    region_for_relation,
    observation_seeds,
    add_exclusions,
    predicted_cells,
)
from loading_perception import inspect_clouds
from loading_contract import validate_thresholds


class LoadingExecutor(SemanticExecutor):
    def __init__(self, **kwargs):
        self.loading = kwargs["config"]["loading_contract"]
        if self.loading != kwargs["config"]["semantic_contract"]["loading_contract"]:
            raise ValueError("Loading rules differ from immutable Body contract")
        validate_thresholds(self.loading["thresholds"])
        self.inspections = []
        self.selected_target = None
        super().__init__(**kwargs)
        self.owner = "daemon_isaac_nav2_loading_area"

    def refresh_catalog(self):
        self.source_check()
        sample = self.fresh()
        now = time.time()
        prior = json.loads((self.root / "loading-known-prior.json").read_text())
        objects = entities(prior)
        relationship = relations(objects)
        map_cfg, _, free, res = free_grid(Path(self.config["semantic_map_yaml"]))
        grid = {
            "resolution": res,
            "width": free.shape[1],
            "height": free.shape[0],
            "origin": map_cfg["origin"][:2],
            "data": np.where(free, 0, 100).ravel().tolist(),
        }
        facilities = [
            b for o in objects if o["category"] == "forklift" for b in o["colliders"]
        ]
        grid = add_exclusions(grid, facilities)
        params = yaml.safe_load(
            (
                Path(self.config["challenge"]) / "config/patrol_navigation_params.yaml"
            ).read_text()
        )
        footprint = json.loads(
            params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        )
        initial = self.contract["return_pose"]
        entries = {}
        targets = []
        latest = self.inspections[-1]["summary"] if self.inspections else None
        observed = (
            set(tuple(p) for p in (latest["free_cells"] + latest["occupied_cells"]))
            if latest
            else set()
        )
        for relation in relationship["ranked"]:
            target = relation["target"]["path"]
            region = region_for_relation(relation)
            proposals = []
            rejected = []
            if (
                region
                and len(self.inspections)
                < self.loading["thresholds"]["max_observations"]
                and relationship["status"] == "RESOLVED"
            ):
                for x, y in observation_seeds(region):
                    center = [(a + b) / 2 for a, b in zip(region["min"], region["max"])]
                    yaw = math.atan2(center[1] - y, center[0] - x)
                    site = {"x": x, "y": y, "yaw": yaw, "target_prim": target}
                    if any(
                        math.dist([x, y], v["site"][:2]) < 1.0 for v in self.inspections
                    ):
                        continue
                    # SIM Body may authorize a restricted observation zone; never expand it.
                    allowed = self.loading.get("allowed_observation_bounds")
                    if allowed and not all(
                        allowed["min"][i] <= site[k] <= allowed["max"][i]
                        for i, k in enumerate(("x", "y"))
                    ):
                        rejected.append(
                            {
                                "xy": [x, y],
                                "reason": "outside immutable allowed observation zone",
                            }
                        )
                        continue
                    try:
                        safe = swept_footprint(
                            [[x, y, yaw]], grid, footprint, padding=0.08
                        )
                    except ValueError as e:
                        rejected.append({"xy": [x, y], "reason": str(e)})
                        continue
                    predicted = predicted_cells(
                        [x, y, yaw], region, self.loading["thresholds"]["resolution_m"]
                    )
                    evidence = {
                        "body_snapshot_hash": self.config["body_snapshot_hash"],
                        "observer_id": self.observer_id,
                        "semantic_contract_sha256": canonical_hash(self.contract),
                        "loading_contract_sha256": canonical_hash(self.loading),
                        "issued_at_wall": now,
                        "issued_at_sim": sample["sim_time"],
                    }
                    ident = canonical_hash({"candidate": site, "evidence": evidence})
                    site["proposal_id"] = ident
                    gain = len(set(tuple(p) for p in predicted) - observed)
                    entries[ident] = {
                        "candidate": site,
                        "evidence": evidence,
                        "kind": "shelf",
                        "generated_at_wall": now,
                        "generated_at_sim": sample["sim_time"],
                        "region": region,
                        "expected_visible_cells": predicted,
                        "static_footprint": safe,
                        "relation": relation,
                        "target_bounds_xy": [region["min"], region["max"]],
                    }
                    proposals.append(
                        {
                            **site,
                            "predicted_new_cells": gain,
                            "distance_from_robot_m": math.dist(
                                [x, y], sample["physics_transforms_xyzw"][0][:2]
                            ),
                        }
                    )
                proposals.sort(
                    key=lambda p: (
                        -p["predicted_new_cells"],
                        p["distance_from_robot_m"],
                    )
                )
            targets.append(
                {
                    "target_prim": target,
                    "shelf_distance_m": relation["shelf_distance_m"],
                    "nearest_shelf": relation["nearest_shelf"]["path"],
                    "region": region,
                    "proposals": proposals[:3],
                    "rejected_count": len(rejected),
                    "rejections": rejected,
                }
            )
        evidence = {
            "body_snapshot_hash": self.config["body_snapshot_hash"],
            "observer_id": self.observer_id,
            "semantic_contract_sha256": canonical_hash(self.contract),
            "issued_at_wall": now,
            "issued_at_sim": sample["sim_time"],
        }
        home = {**initial, "target_prim": "return_to_initial_pose"}
        ident = canonical_hash({"candidate": home, "evidence": evidence})
        home["proposal_id"] = ident
        entries[ident] = {
            "candidate": home,
            "evidence": evidence,
            "kind": "return",
            "generated_at_wall": now,
            "generated_at_sim": sample["sim_time"],
        }
        with self.registry_lock:
            self.registry.update(entries)
        write_json_atomic(
            self.root / "loading-catalog.json",
            {
                "targets": targets,
                "relation_status": relationship["status"],
                "relation_method": relationship["method"],
                "return_to_initial_pose": home,
                "initial_pose": initial,
                "generated_at_wall": now,
                "body_snapshot_hash": self.config["body_snapshot_hash"],
                "inspection_count": len(self.inspections),
                "latest_inspection": latest,
                "requirements": [
                    "Choose the forklift NEAR SHELVES using geometry; not nearest to robot.",
                    "Request one immutable proposal at a time. After each visit read actual measured inspection.",
                    "If UNKNOWN or coverage below 95% and safe unused candidates remain, choose a second with predicted new coverage. Max two views. Do not call low-gain repositioning a successful active observation.",
                    "CLEAR, OBSTRUCTED and UNKNOWN are inspection results, separate from task success.",
                    "Return to actual initial pose, then verify_and_remember with canonical action_ids and inspection_report.",
                    "No scene truth/file/ground-truth access. Scene prior supplies facilities only; sensor measures temporary occupancy.",
                ],
                "evidence_domain": "SIMULATION",
                "authorization": False,
            },
        )

    def probe(self, site, *, sensor_only=False):
        snapshot = super().probe(site, sensor_only=sensor_only)
        if not sensor_only:
            prior = json.loads((self.root / "loading-known-prior.json").read_text())
            boxes = [
                b
                for o in entities(prior)
                if o["category"] == "forklift"
                for b in o["colliders"]
            ]
            snapshot["costmap"] = add_exclusions(snapshot["costmap"], boxes)
            snapshot["known_facility_exclusions"] = boxes
        return snapshot

    def resolve_target(self, action):
        with self.registry_lock:
            entry = self.registry.get(action.arguments["proposal_id"])
        if entry and entry["kind"] != "return":
            if len(self.inspections) >= self.loading["thresholds"]["max_observations"]:
                raise ValueError("Observation budget exhausted")
            if (
                self.selected_target
                and entry["candidate"]["target_prim"] != self.selected_target
            ):
                raise ValueError("Cannot silently switch inspection target")
        return super().resolve_target(action)

    def enrich_verification(self, action, site_id, site, proof):
        semantic = self.resolved[action.action_id]
        if semantic["proposal"]["kind"] == "return":
            return {
                **proof,
                "semantic": semantic,
                "inspection_scope": "loading-area return",
            }
        ident = semantic["proposal_id"]
        out = self.evidence_dir / (ident + "-cloud.json")
        # The ephemeral read-only ROS probe exports actual data over stdout.
        container_output = "/tmp/loading-cloud-" + ident + ".json"
        # A single ephemeral container must export the measured payload before exit.
        command = [
            str(Path(self.config["challenge"]) / "scripts/ros-container.sh"),
            "bash",
            "-c",
            'python3 /lab/ros2/loading_sensor_probe.py --frames 3 --output "$1" >&2 && cat "$1"',
            "bash",
            container_output,
        ]
        data = subprocess.run(
            command, capture_output=True, text=True, timeout=75, check=True
        )
        snapshot = json.loads(data.stdout)
        physical = self.fresh()
        last = snapshot["frames"][-1]
        if (
            abs(physical["sim_time"] - last["stamp"]) > 1.5
            or time.time() - snapshot["wall_time"] > 3
        ):
            raise ValueError("Stale perception/independent physics")
        if (
            math.hypot(*physical["linear_velocity_xyz"][:2]) > 0.02
            or abs(physical["angular_velocity_xyz"][2]) > 0.1
        ):
            raise ValueError("Robot not stopped during inspection")
        q = physical["physics_transforms_xyzw"][0]
        yaw = math.atan2(
            2 * (q[6] * q[5] + q[3] * q[4]), 1 - 2 * (q[4] * q[4] + q[5] * q[5])
        )
        if (
            math.dist(q[:2], [site["x"], site["y"]]) > 0.4
            or abs(angle_delta(yaw, site["yaw"])) > 0.35
        ):
            raise ValueError("Inspection physical pose mismatch")
        prior = json.loads((self.root / "loading-known-prior.json").read_text())
        facilities = [o for o in entities(prior)]
        summary = inspect_clouds(
            snapshot,
            semantic["proposal"]["region"],
            thresholds=self.loading["thresholds"],
            previous=self.inspections[-1]["summary"] if self.inspections else None,
            known_facilities=facilities,
        )
        write_json_atomic(
            out,
            {"snapshot": snapshot, "independent_physics": physical, "summary": summary},
        )
        artifact = {"path": str(out), "sha256": file_sha(out)}
        self.inspections.append(
            {
                "action_id": action.action_id,
                "site": [site[k] for k in ("x", "y", "yaw")],
                "summary": summary,
                "artifact": artifact,
            }
        )
        self.selected_target = semantic["proposal"]["candidate"]["target_prim"]
        self.refresh_catalog()
        return {
            **proof,
            "semantic": {**semantic, "inspection_artifact": artifact},
            "inspection_scope": "actual ground/height-band PointCloud2",
            "inspection_result": {
                k: v
                for k, v in summary.items()
                if k
                not in [
                    "free_cells",
                    "occupied_cells",
                    "region",
                    "thresholds",
                    "clearance_halo",
                    "clearance_safe_center_cells",
                    "clearance_blocked_center_cells",
                    "height_band_mask_this_view",
                    "accumulated_height_band_mask",
                ]
            },
        }


class LoadingMemoryExecutor(SemanticMemoryExecutor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.agent_report = None
        self.replay_summary = None
        self.replay_views = []

    def validate_arguments(self, action):
        if set(action.arguments) != {"action_ids", "inspection_report"}:
            raise ValueError("Action IDs and Agent inspection report required")
        report = action.arguments["inspection_report"]
        fields = {
            "target_id",
            "result",
            "extra_obstacle_detected",
            "uncertainties",
            "summary",
        }
        if not isinstance(report, dict) or set(report) != fields:
            raise ValueError(
                "Report requires target_id/result/extra_obstacle_detected/uncertainties/summary"
            )
        if (
            report["result"] not in {"CLEAR", "OBSTRUCTED", "UNKNOWN"}
            or type(report["extra_obstacle_detected"]) is not bool
        ):
            raise ValueError("Invalid inspection conclusion")
        if not isinstance(report["uncertainties"], list) or not all(
            isinstance(x, str) for x in report["uncertainties"]
        ):
            raise ValueError("Explicit uncertainties required")
        if not isinstance(report["summary"], str) or not report["summary"].strip():
            raise ValueError("Agent summary required")
        self.agent_report = report
        self.replay_summary = None
        self.replay_views = []

    def check_extra_evidence(self, data, proof):
        super().check_extra_evidence(data, proof)
        semantic = data["verification"]["semantic"]
        if semantic["proposal"]["kind"] == "return":
            return
        cloud = checked_artifact(
            self.root, self.config, semantic["inspection_artifact"]
        )
        physical = cloud["independent_physics"]
        snapshot = cloud["snapshot"]
        last = snapshot["frames"][-1]
        if (
            physical["observer_id"]
            != self.config["semantic_contract"]["initial_observer_id"]
            or abs(physical["sim_time"] - last["stamp"]) > 1.5
        ):
            raise ValueError("Independent inspection time/observer mismatch")
        if (
            physical["collision_count"]
            or not physical["collision_observer_complete"]
            or physical["contact_errors"]
        ):
            raise ValueError("Inspection contact gate failed")
        if (
            math.hypot(*physical["linear_velocity_xyz"][:2]) > 0.02
            or abs(physical["angular_velocity_xyz"][2]) > 0.1
        ):
            raise ValueError("Inspection was not stopped")
        prior = json.loads((self.root / "loading-known-prior.json").read_text())
        expected = inspect_clouds(
            snapshot,
            semantic["proposal"]["region"],
            thresholds=self.config["loading_contract"]["thresholds"],
            previous=self.replay_summary,
            known_facilities=entities(prior),
        )
        if expected != cloud["summary"]:
            raise ValueError("Measured inspection replay mismatch")
        self.replay_summary = expected
        self.replay_views.append(
            {
                "site": data["site"],
                "action_id": data["action_id"],
                "new_observed_cells": expected["new_observed_cells"],
                "coverage_ratio": expected["coverage_ratio"],
            }
        )

    def check_mission(self, visits):
        if not 1 <= len(visits) <= 3:
            raise ValueError("At most two observations and one return")
        entries = [
            json.loads(
                __import__("artifacts")
                .artifact_path(
                    self.root, self.config, v["artifact"], directory="actions"
                )
                .read_text()
            )["verification"]["semantic"]["proposal"]
            for v in visits
        ]
        if entries[-1]["kind"] != "return" or any(
            e["kind"] == "return" for e in entries[:-1]
        ):
            raise ValueError("Final return required")
        prior = json.loads((self.root / "loading-known-prior.json").read_text())
        prior_source = self.config["loading_contract"]["known_prior_source"]
        if (
            file_sha(self.root / "loading-known-prior.json")
            != self.config["semantic_contract"]["source_sha256"][prior_source]
        ):
            raise ValueError("Known facility prior hash mismatch")
        relation = relations(entities(prior))
        expected_target = relation["ranked"][0]["target"]["path"]
        if relation["status"] != "RESOLVED":
            raise ValueError(
                "Ambiguous relation requires separate clarification workflow"
            )
        if any(e["candidate"]["target_prim"] != expected_target for e in entries[:-1]):
            raise ValueError("Wrong forklift relation")
        if len(entries) == 1:
            # Restricted Body must make every generated observation inadmissible.
            allowed = self.config["loading_contract"].get("allowed_observation_bounds")
            region = region_for_relation(relation["ranked"][0])
            if not allowed or any(
                all(allowed["min"][i] <= p[i] <= allowed["max"][i] for i in (0, 1))
                for p in observation_seeds(region)
            ):
                raise ValueError("Cannot skip available observation candidates")
            self.replay_summary = {
                "result": "UNKNOWN",
                "extra_obstacle_detected": False,
                "coverage_ratio": 0.0,
                "reason": "No generated observation lies in immutable authorized observation zone",
                "new_observed_cells": 0,
            }
        if self.agent_report:
            report = self.agent_report
            if (
                report["target_id"] != expected_target
                or report["result"] != self.replay_summary["result"]
                or report["extra_obstacle_detected"]
                != self.replay_summary["extra_obstacle_detected"]
            ):
                raise ValueError("Agent conclusion differs from measured evidence")
            if report["result"] == "UNKNOWN" and not report["uncertainties"]:
                raise ValueError("UNKNOWN must state uncertainty")
        if (
            len(self.replay_views) == 2
            and math.dist(
                [self.replay_views[0]["site"][k] for k in ("x", "y")],
                [self.replay_views[1]["site"][k] for k in ("x", "y")],
            )
            < 1
        ):
            raise ValueError("Repeated viewpoint is not active observation")

    def verification_extras(self):
        return {
            "loading_inspection": self.replay_summary,
            "inspection_report": self.agent_report,
            "observation_views": self.replay_views,
            "task_success_separate_from_inspection_result": True,
        }
