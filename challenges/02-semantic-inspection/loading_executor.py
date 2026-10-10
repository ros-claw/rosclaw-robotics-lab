"""Loading-area single-goal executor reusing immutable proposal, Nav2 and receipts."""

import json
import math
from pathlib import Path
import subprocess
import time
import threading
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
from loading_path import preview_candidates, verify_refused_candidates


class CatalogPreviewInterrupted(Exception):
    """A motion request takes precedence over a background read-only refresh."""


class LoadingExecutor(SemanticExecutor):
    def __init__(self, **kwargs):
        self.loading = kwargs["config"]["loading_contract"]
        if self.loading != kwargs["config"]["semantic_contract"]["loading_contract"]:
            raise ValueError("Loading rules differ from immutable Body contract")
        validate_thresholds(self.loading["thresholds"])
        self.loading_catalog_lock = threading.RLock()
        self.loading_planning_lock = threading.Lock()
        self.inspections = []
        self.selected_target = None
        super().__init__(**kwargs)
        self.owner = "daemon_isaac_nav2_loading_area"

    def refresh_catalog(self, *, allow_during_action=False):
        # Do not compete with a live navigation action for the Nav2 planner.
        if self.lock.locked() and not allow_during_action:
            return
        with self.loading_catalog_lock:
            return self._refresh_loading_catalog(
                allow_during_action=allow_during_action
            )

    def catalog_loop(self):
        while not self.catalog_stop.wait(5):
            if self.lock.locked():
                continue
            try:
                catalog = json.loads((self.root / "loading-catalog.json").read_text())
                if time.time() - catalog["generated_at_wall"] >= 45:
                    self.refresh_catalog()
            except CatalogPreviewInterrupted:
                continue
            except Exception as exc:
                write_json_atomic(
                    self.root / "semantic-catalog-error.json",
                    {"wall_time": time.time(), "error": str(exc)},
                )

    def _refresh_loading_catalog(self, *, allow_during_action=False):
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
        all_rejected_path_artifacts = {}
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

            def preview_probe(candidate):
                if self.lock.locked() and not allow_during_action:
                    raise CatalogPreviewInterrupted()
                return self.probe(candidate)

            accepted, previews = preview_candidates(
                proposals,
                entries,
                preview_probe,
                footprint,
                on_record=lambda record: write_json_atomic(
                    self.evidence_dir
                    / (
                        record["proposal"]["candidate"]["proposal_id"] + "-preview.json"
                    ),
                    record,
                ),
            )
            checked_proposals = [{**c, "actual_path_preview": "PASS"} for c in accepted]
            for preview in previews:
                entry = preview["proposal"]
                candidate = entry["candidate"]
                ident = candidate["proposal_id"]
                if preview["status"] == "PASS":
                    entries.pop(ident)
                    issued = self.fresh()
                    entry["evidence"].update(
                        issued_at_wall=time.time(), issued_at_sim=issued["sim_time"]
                    )
                    raw = {k: v for k, v in candidate.items() if k != "proposal_id"}
                    ident = canonical_hash(
                        {"candidate": raw, "evidence": entry["evidence"]}
                    )
                    candidate["proposal_id"] = ident
                    entry.update(
                        generated_at_wall=entry["evidence"]["issued_at_wall"],
                        generated_at_sim=issued["sim_time"],
                    )
                    entries[ident] = entry
                    original = next(
                        c
                        for c in accepted
                        if c["x"] == candidate["x"] and c["y"] == candidate["y"]
                    )
                    checked_proposals.append(
                        {
                            **original,
                            "proposal_id": ident,
                            "actual_path_preview": "PASS",
                            "path_preview_wall_time": preview["snapshot"]["wall_time"],
                        }
                    )
                artifact_path = self.evidence_dir / (ident + "-preview.json")
                write_json_atomic(artifact_path, preview)
                if preview["status"] != "PASS":
                    rejected.append(
                        {
                            "xy": [candidate["x"], candidate["y"]],
                            "reason": "actual Nav2 path rejected: " + preview["error"],
                        }
                    )
                    entries.pop(ident)
                    if "snapshot" in preview and "checked_path" in preview:
                        all_rejected_path_artifacts.setdefault(target, []).append(
                            {
                                "path": str(artifact_path),
                                "sha256": file_sha(artifact_path),
                            }
                        )
                else:
                    entry["path_preview_artifact"] = {
                        "path": str(artifact_path),
                        "sha256": file_sha(artifact_path),
                    }
            targets.append(
                {
                    "target_prim": target,
                    "shelf_distance_m": relation["shelf_distance_m"],
                    "nearest_shelf": relation["nearest_shelf"]["path"],
                    "region": region,
                    "proposals": checked_proposals,
                    "rejected_count": len(rejected),
                    "rejections": rejected,
                }
            )
        # Return proposal and catalog lifetime begin AFTER path previews finish.
        sample = self.fresh()
        now = time.time()
        unavailable = None
        nearest = targets[0]
        rejection_proofs = all_rejected_path_artifacts.get(nearest["target_prim"], [])
        if (
            not nearest["proposals"]
            and nearest["region"]
            and len(rejection_proofs) == len(observation_seeds(nearest["region"]))
        ):
            refusal = {
                "body_snapshot_hash": self.config["body_snapshot_hash"],
                "observer_id": self.observer_id,
                "target": nearest["target_prim"],
                "region": nearest["region"],
                "path_rejections": rejection_proofs,
                "reason": "Every generated observation path failed actual full-footprint checks; this bounded candidate set cannot support inspection",
            }
            path = self.evidence_dir / (
                "unavailable-" + canonical_hash(refusal) + ".json"
            )
            write_json_atomic(path, refusal)
            unavailable = {"path": str(path), "sha256": file_sha(path)}
        evidence = {
            "body_snapshot_hash": self.config["body_snapshot_hash"],
            "observer_id": self.observer_id,
            "semantic_contract_sha256": canonical_hash(self.contract),
            "issued_at_wall": now,
            "issued_at_sim": sample["sim_time"],
        }
        if unavailable:
            evidence["observation_unavailable_sha256"] = unavailable["sha256"]
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
        if unavailable:
            entries[ident]["observation_unavailable_artifact"] = unavailable
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
        with self.loading_planning_lock:
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
        # Registry is immutable; do not wait for an entire background preview batch.
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
        self.refresh_catalog(allow_during_action=True)
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
        preview = checked_artifact(
            self.root, self.config, semantic["proposal"]["path_preview_artifact"]
        )
        if (
            preview["status"] != "PASS"
            or preview["proposal"]["candidate"] != semantic["proposal"]["candidate"]
        ):
            raise ValueError("Actual candidate path preview mismatch")
        params = yaml.safe_load(
            (
                self.root / "frozen-source/config/patrol_navigation_params.yaml"
            ).read_text()
        )
        footprint = json.loads(
            params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        )
        swept_footprint(
            preview["checked_path"],
            preview["snapshot"]["costmap"],
            footprint,
            padding=0.08,
        )
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
            zone_refusal = allowed and not any(
                all(allowed["min"][i] <= p[i] <= allowed["max"][i] for i in (0, 1))
                for p in observation_seeds(region)
            )
            refusal_reason = (
                "No generated observation lies in immutable authorized observation zone"
            )
            if not zone_refusal:
                artifact = entries[-1].get("observation_unavailable_artifact")
                if (
                    not artifact
                    or entries[-1]["evidence"].get("observation_unavailable_sha256")
                    != artifact["sha256"]
                ):
                    raise ValueError("Cannot skip available observation candidates")
                refusal = checked_artifact(self.root, self.config, artifact)
                if (
                    refusal["body_snapshot_hash"] != self.config["body_snapshot_hash"]
                    or refusal["observer_id"]
                    != self.config["semantic_contract"]["initial_observer_id"]
                    or refusal["target"] != expected_target
                    or refusal["region"] != region
                ):
                    raise ValueError("Unsafe-path refusal binding mismatch")
                params = yaml.safe_load(
                    (
                        self.root / "frozen-source/config/patrol_navigation_params.yaml"
                    ).read_text()
                )
                footprint = json.loads(
                    params["local_costmap"]["local_costmap"]["ros__parameters"][
                        "footprint"
                    ]
                )
                verify_refused_candidates(
                    [
                        checked_artifact(self.root, self.config, item)
                        for item in refusal["path_rejections"]
                    ],
                    observation_seeds(region),
                    footprint,
                    expected_target,
                )
                refusal_reason = refusal["reason"]
            self.replay_summary = {
                "result": "UNKNOWN",
                "extra_obstacle_detected": False,
                "coverage_ratio": 0.0,
                "reason": refusal_reason,
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
