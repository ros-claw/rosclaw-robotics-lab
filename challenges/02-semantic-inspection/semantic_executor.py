"""Generated SIM targets inside the existing Nav2 daemon executor and Memory loop."""

import hashlib
import json
import math
from pathlib import Path
import subprocess
import threading
import time

from rosclaw.body.resolver import BodyResolver
from rosclaw.kernel import ExecutionMode
from point_executor import PointExecutor
from remember import PatrolMemoryExecutor
from evidence_io import write_json_atomic
from propose_observation import propose
from safety import canonical_hash, swept_footprint, lidar_target_hits


def file_sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def checked_artifact(root, artifact):
    path = Path(artifact["path"]).resolve()
    if (
        not path.is_relative_to((Path(root) / "semantic-evidence").resolve())
        or file_sha(path) != artifact["sha256"]
    ):
        raise ValueError("Semantic evidence artifact path/hash mismatch")
    return json.loads(path.read_text())


class SemanticExecutor(PointExecutor):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.owner = "daemon_isaac_nav2_immutable_semantic_proposal"
        self.contract = self.config["semantic_contract"]
        effective = BodyResolver(workspace=self.root / "home").get_effective_body()
        if (
            effective.compute_hash() != self.config["body_snapshot_hash"]
            or effective.safety.get("navigation_contract") != self.contract
        ):
            raise ValueError("Semantic contract differs from bound immutable Body")
        self.source_check()
        self.registry = {}
        self.registry_lock = threading.Lock()
        self.consumed = set()
        self.resolved = {}
        self.catalog_stop = threading.Event()
        self.evidence_dir = self.root / "semantic-evidence"
        self.evidence_dir.mkdir(exist_ok=True)
        self.refresh_catalog()
        self.catalog_thread = threading.Thread(target=self.catalog_loop, daemon=True)
        self.catalog_thread.start()

    def source_check(self):
        if (
            self.contract.get("real_allowed") is not False
            or self.contract.get("kind") != "immutable_semantic_proposal_SIM_v1"
        ):
            raise ValueError("Only explicit generated-target SIM contract supported")
        for p, h in self.contract["source_sha256"].items():
            if file_sha(p) != h:
                raise ValueError("Bound semantic source changed: " + str(p))
        if self.fresh()["observer_id"] != self.contract["initial_observer_id"]:
            raise ValueError("Semantic observer differs from bound reset")

    def refresh_catalog(self):
        self.source_check()
        targets = []
        rejections = []
        entries = {}
        inventory = json.loads(
            (
                Path(self.config["physics_directory"]) / "stage-inventory.json"
            ).read_text()
        )
        rows = {r["path"]: r for r in inventory["scene_objects"]}
        initial = self.contract["return_pose"]
        for target in self.contract["allowed_development_targets"]:
            sample = self.fresh()
            seed = self.evidence_dir / ("seed-" + str(sample["sequence"]) + ".json")
            write_json_atomic(seed, sample)
            try:
                result = propose(
                    inventory_path=Path(self.config["physics_directory"])
                    / "stage-inventory.json",
                    map_yaml=Path(self.config["semantic_map_yaml"]),
                    physics_path=seed,
                    body_path=self.root / "body.json",
                    nav_params=Path(self.config["challenge"])
                    / "config/patrol_navigation_params.yaml",
                    target=target,
                )
                row = rows[target]
                center = [(a + b) / 2 for a, b in zip(row["min"][:2], row["max"][:2])]
                for c in result["candidates"]:
                    entries[c["proposal_id"]] = {
                        "candidate": c,
                        "evidence": result["evidence"],
                        "target_bounds_xy": result["target_bounds_xy"],
                        "generated_at_wall": result["generated_at_wall"],
                        "generated_at_sim": sample["sim_time"],
                        "kind": "shelf",
                        "seed_artifact": {"path": str(seed), "sha256": file_sha(seed)},
                    }
                targets.append(
                    {
                        "target_prim": target,
                        "center_xy": center,
                        "bounds_xy": result["target_bounds_xy"],
                        "distance_from_initial_pose_m": math.dist(
                            center, [initial["x"], initial["y"]]
                        ),
                        "west_of_initial_pose": center[0] < initial["x"],
                        "status": result["status"],
                        "proposals": result["candidates"],
                    }
                )
            except ValueError as exc:
                rejections.append({"target_prim": target, "reason": str(exc)})
        sample = self.fresh()
        now = time.time()
        home = {
            "x": initial["x"],
            "y": initial["y"],
            "yaw": initial["yaw"],
            "target_prim": "return_to_initial_pose",
        }
        evidence = {
            "body_snapshot_hash": self.config["body_snapshot_hash"],
            "observer_id": self.observer_id,
            "semantic_contract_sha256": canonical_hash(self.contract),
            "issued_at_wall": now,
            "issued_at_sim": sample["sim_time"],
        }
        ident = canonical_hash({"candidate": home, "evidence": evidence})
        home["proposal_id"] = ident
        entries[ident] = {
            "candidate": home,
            "evidence": evidence,
            "generated_at_wall": now,
            "generated_at_sim": sample["sim_time"],
            "kind": "return",
        }
        with self.registry_lock:
            self.registry.update(entries)
        write_json_atomic(
            self.root / "semantic-catalog.json",
            {
                "schema_version": "rosclaw.semantic_catalog.v1",
                "generated_at_wall": now,
                "body_id": self.config["body_id"],
                "body_snapshot_hash": self.config["body_snapshot_hash"],
                "initial_pose": initial,
                "targets": targets,
                "return_to_initial_pose": home,
                "rejections": rejections,
                "proposal_max_wall_age_seconds": self.contract[
                    "proposal_max_wall_age_seconds"
                ],
                "semantic_source": "USD-known development shelves; no sensor semantic discovery",
                "inspection_scope": self.contract["inspection_scope"],
                "holdout_exposed": False,
            },
        )

    def catalog_loop(self):
        while not self.catalog_stop.wait(15):
            try:
                self.refresh_catalog()
            except Exception as exc:
                write_json_atomic(
                    self.root / "semantic-catalog-error.json",
                    {"wall_time": time.time(), "error": str(exc)},
                )

    def close(self):
        self.catalog_stop.set()
        self.catalog_thread.join(timeout=10)

    def validate_action(self, action):
        return (
            action.execution_mode is ExecutionMode.SIMULATION
            and action.body_id == self.config["body_id"]
            and action.body_snapshot_hash == self.config["body_snapshot_hash"]
            and action.capability_id == "navigation.navigate_to_pose"
            and set(action.arguments) == {"proposal_id"}
            and isinstance(action.arguments["proposal_id"], str)
        )

    def probe(self, site, *, sensor_only=False):
        command = [
            str(Path(self.config["challenge"]) / "scripts/ros-container.sh"),
            "python3",
            "/lab/ros2/semantic_path_probe.py",
            "--goal",
            json.dumps({k: site[k] for k in ["x", "y", "yaw"]}),
        ]
        if sensor_only:
            command.append("--sensor-only")
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=40, check=True
        )
        return json.loads(result.stdout)

    def resolve_target(self, action):
        self.source_check()
        ident = action.arguments["proposal_id"]
        with self.registry_lock:
            if ident not in self.registry or ident in self.consumed:
                raise ValueError("Unknown or consumed daemon-issued proposal")
            entry = self.registry[ident]
        sample = self.fresh()
        if (
            not 0
            <= time.time() - entry["generated_at_wall"]
            <= self.contract["proposal_max_wall_age_seconds"]
            or not 0
            <= sample["sim_time"] - entry["generated_at_sim"]
            <= self.contract["proposal_max_sim_age_seconds"]
        ):
            raise ValueError(
                "Proposal expired; observe current candidates and explicitly choose again"
            )
        site = {k: entry["candidate"][k] for k in ["x", "y", "yaw"]}
        snapshot = self.probe(site)
        sample = self.fresh()
        if (
            not 0 <= time.time() - snapshot["wall_time"] <= 3
            or abs(sample["sim_time"] - snapshot["sim_time"]) > 1.5
        ):
            raise ValueError("Planner snapshot is stale at dispatch")
        if (
            math.dist(
                sample["physics_transforms_xyzw"][0][:2], snapshot["base_pose"][:2]
            )
            > 0.5
        ):
            raise ValueError("Independent PhysX and map TF disagree")
        if (
            math.dist(snapshot["path"][0][:2], snapshot["base_pose"][:2]) > 0.5
            or math.dist(snapshot["path"][-1][:2], [site["x"], site["y"]]) > 0.4
        ):
            raise ValueError("Path does not bind current pose and exact target")
        footprint = json.loads(
            __import__("yaml").safe_load(
                (
                    Path(self.config["challenge"])
                    / "config/patrol_navigation_params.yaml"
                ).read_text()
            )["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        )
        path = [
            snapshot["base_pose"],
            *snapshot["path"],
            [site["x"], site["y"], site["yaw"]],
        ]
        proof = swept_footprint(
            path,
            snapshot["costmap"],
            footprint,
            padding=self.contract["footprint_padding_and_margin_m"],
        )
        if self.stopping.is_set():
            raise ValueError("Stop requested during planning")
        self.source_check()
        target = self.evidence_dir / (ident + "-plan.json")
        write_json_atomic(
            target,
            {
                "proposal": entry,
                "snapshot": snapshot,
                "swept_footprint": proof,
                "checked_path": path,
            },
        )
        with self.registry_lock:
            self.consumed.add(ident)
        self.resolved[action.action_id] = {
            "proposal": entry,
            "proposal_id": ident,
            "plan_artifact": {"path": str(target), "sha256": file_sha(target)},
        }
        return ident, site

    def enrich_verification(self, action, site_id, site, proof):
        semantic = self.resolved[action.action_id]
        if (
            self.config.get("require_target_lidar")
            and semantic["proposal"]["kind"] == "shelf"
        ):
            snapshot = self.probe(site, sensor_only=True)
            hit_proof = lidar_target_hits(
                snapshot["scan"],
                semantic["proposal"]["target_bounds_xy"],
                sim_time=snapshot["sim_time"],
            )
            target = self.evidence_dir / (site_id + "-inspection.json")
            write_json_atomic(target, {"snapshot": snapshot, "verification": hit_proof})
            semantic = {
                **semantic,
                "inspection_artifact": {
                    "path": str(target),
                    "sha256": file_sha(target),
                },
            }
        return {
            **proof,
            "semantic": semantic,
            "inspection_scope": self.contract["inspection_scope"],
        }


class SemanticMemoryExecutor(PatrolMemoryExecutor):
    def target_for_visit(self, data):
        entry = data["verification"]["semantic"]["proposal"]
        candidate = entry["candidate"]
        ident = candidate["proposal_id"]
        raw = {k: v for k, v in candidate.items() if k != "proposal_id"}
        if (
            canonical_hash({"candidate": raw, "evidence": entry["evidence"]}) != ident
            or ident != data["site_id"]
        ):
            raise ValueError("Proposal content/ID mismatch")
        if (
            entry["kind"] == "shelf"
            and candidate["target_prim"]
            not in self.config["semantic_contract"]["allowed_development_targets"]
        ):
            raise ValueError("Target outside immutable Body contract")
        if entry["evidence"]["body_snapshot_hash"] != self.config["body_snapshot_hash"]:
            raise ValueError("Proposal bound to another Body")
        if (
            entry["kind"] == "return"
            and {k: candidate[k] for k in ["x", "y", "yaw"]}
            != self.config["semantic_contract"]["return_pose"]
        ):
            raise ValueError("Return target differs from actual initial pose")
        if data["site"] != {k: candidate[k] for k in ["x", "y", "yaw"]}:
            raise ValueError("Executed pose differs from immutable proposal")
        return data["site"]

    def check_extra_evidence(self, data, proof):
        semantic = data["verification"]["semantic"]
        plan = checked_artifact(self.root, semantic["plan_artifact"])
        if plan["proposal"] != semantic["proposal"]:
            raise ValueError("Planning proposal mismatch")
        import yaml

        params = yaml.safe_load(
            (
                Path(self.config["challenge"]) / "config/patrol_navigation_params.yaml"
            ).read_text()
        )
        footprint = json.loads(
            params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        )
        swept_footprint(
            plan["checked_path"],
            plan["snapshot"]["costmap"],
            footprint,
            padding=self.config["semantic_contract"]["footprint_padding_and_margin_m"],
        )
        if (
            self.config.get("require_target_lidar")
            and semantic["proposal"]["kind"] == "shelf"
        ):
            inspection = checked_artifact(self.root, semantic["inspection_artifact"])
            lidar_target_hits(
                inspection["snapshot"]["scan"],
                semantic["proposal"]["target_bounds_xy"],
                sim_time=inspection["snapshot"]["sim_time"],
            )

    def check_mission(self, visits):
        if len(visits) != 2:
            raise ValueError(
                "Pilot requires exactly one requested shelf and return to actual initial pose"
            )
        entries = [
            json.loads(Path(v["artifact"]["path"]).read_text())["verification"][
                "semantic"
            ]["proposal"]
            for v in visits
        ]
        if entries[0]["kind"] != "shelf" or entries[1]["kind"] != "return":
            raise ValueError("Shelf/return order mismatch")
        inventory = json.loads(
            (
                Path(self.config["physics_directory"]) / "stage-inventory.json"
            ).read_text()
        )
        initial = self.config["semantic_contract"]["return_pose"]
        allowed = self.config["semantic_contract"]["allowed_development_targets"]
        rows = []
        for row in inventory["scene_objects"]:
            if row["path"] not in allowed:
                continue
            center = [(a + b) / 2 for a, b in zip(row["min"][:2], row["max"][:2])]
            if center[0] < initial["x"]:
                rows.append(
                    (math.dist(center, [initial["x"], initial["y"]]), row["path"])
                )
        if not rows or entries[0]["candidate"]["target_prim"] != min(rows)[1]:
            raise ValueError(
                "Selected target does not satisfy nearest shelf west of actual initial pose"
            )
