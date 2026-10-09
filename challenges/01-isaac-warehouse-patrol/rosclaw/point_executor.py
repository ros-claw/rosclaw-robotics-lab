"""Daemon-owned, SIM-only single-site Nav2 adapter. No patrol sequence here."""

import hashlib
import json
import math
import subprocess
import threading
import time
from pathlib import Path

from rosclaw.kernel import (
    ActionExecutionResult,
    ActionState,
    EvidenceDomain,
    EvidenceLevel,
    ExecutionMode,
)
from patrol import verify_visit
from evidence_io import write_json_atomic


class ScanWitness:
    def __init__(self, transport):
        self.transport = transport
        self.latest = None
        self.lock = threading.Lock()
        self.closed = threading.Event()
        response = transport.send(
            {
                "op": "subscribe",
                "id": "isaac-scan-witness",
                "topic": "/scan",
                "type": "sensor_msgs/msg/LaserScan",
                "throttle_rate": 300,
                "queue_length": 1,
            }
        )
        if not response.ok:
            raise RuntimeError(response.error)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        while not self.closed.is_set():
            response = self.transport.receive(timeout_sec=0.3)
            if not response.ok or (response.data or {}).get("topic") != "/scan":
                continue
            msg = response.data["msg"]
            header = msg.get("header", {})
            stamp = header.get("stamp", {})
            hits = sum(
                isinstance(value, (int, float)) and math.isfinite(value) and value > 0
                for value in msg.get("ranges", [])
            )
            sample = {
                "wall_time": time.time(),
                "stamp": stamp.get("sec", 0) + stamp.get("nanosec", 0) / 1e9,
                "frame_id": header.get("frame_id"),
                "range_count": len(msg.get("ranges", [])),
                "finite_hits": hits,
                "valid": header.get("frame_id") == "front_3d_lidar" and hits > 0,
            }
            with self.lock:
                self.latest = sample

    def snapshot(self):
        with self.lock:
            return dict(self.latest) if self.latest else None

    def close(self):
        self.closed.set()
        self.transport.close()
        self.thread.join(timeout=2)


class PointExecutor:
    def __init__(self, *, root, config, client, sensor):
        self.root, self.config, self.client, self.sensor = (
            Path(root),
            config,
            client,
            sensor,
        )
        self.physics = Path(config["physics_directory"]) / "physics-latest.json"
        self.body_path = config["physics_body_path"]
        self.observer_id = json.loads(self.physics.read_text()).get("observer_id")
        self.lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_id = None
        self.stopping = threading.Event()
        self.owner = "daemon_isaac_nav2_single_site"
        self.output = self.root / "actions"
        self.output.mkdir(exist_ok=True)

    def fresh(self):
        sample = json.loads(self.physics.read_text())
        if self.observer_id and sample.get("observer_id") != self.observer_id:
            raise RuntimeError("simulation observer changed during mission")
        if not 0 <= time.time() - sample["wall_time"] <= 2:
            raise RuntimeError("independent physics is stale")
        if sample["physics_body_path"] != self.body_path or not sample.get(
            "timeline_playing"
        ):
            raise RuntimeError("simulation paused or physical Body differs")
        if not sample.get("collision_observer_complete") or sample.get(
            "contact_errors"
        ):
            raise RuntimeError("collision observation is incomplete")
        if sample.get("collision_count"):
            raise RuntimeError("non-floor collision observed")
        return sample

    def emergency_stop(self):
        self.stopping.set()
        with self.goal_lock:
            goal = self.goal_id
        if goal:
            self.client.cancel_goal(goal)
        try:
            sample = self.fresh()
            stopped = (
                math.hypot(*sample["linear_velocity_xyz"][:2]) <= 0.02
                and abs(sample["angular_velocity_xyz"][2]) <= 0.1
            )
        except Exception:
            stopped = False
        return {"acknowledged": True, "physical_stop_verified": stopped}

    def __call__(self, action):
        if (
            action.execution_mode is not ExecutionMode.SIMULATION
            or action.body_id != self.config["body_id"]
            or action.body_snapshot_hash != self.config["body_snapshot_hash"]
            or action.capability_id != "navigation.navigate_to_pose"
            or set(action.arguments) != {"site_id"}
            or action.arguments.get("site_id") not in self.config["sites"]
        ):
            return self.result(
                ActionState.BLOCKED,
                error="SIM mode, Body binding or registered site invalid",
            )
        if not self.lock.acquire(blocking=False):
            return self.result(
                ActionState.BLOCKED, error="another navigation action is active"
            )
        try:
            return self.execute(action)
        finally:
            self.lock.release()

    def execute(self, action):
        path = self.output / (
            hashlib.sha256(action.action_id.encode()).hexdigest()[:24] + ".json"
        )
        events, samples = [], []
        done, nav = threading.Event(), {}
        goal_id = "isaac-" + hashlib.sha256(action.action_id.encode()).hexdigest()[:24]
        started = time.time()
        deadline = min(
            time.monotonic() + 420,
            time.monotonic() + max(0, action.deadline_at.timestamp() - time.time()),
        )
        site_id = action.arguments["site_id"]
        site = self.config["sites"][site_id]
        proof = None
        last_sim_change = time.monotonic()

        def event(kind, **data):
            events.append({"wall_time": time.time(), "event": kind, **data})

        def feedback(data):
            recoveries = int(data.get("number_of_recoveries", 0))
            event("feedback", values=data)
            if recoveries > self.config["maximum_recoveries"]:
                nav["recovery_limit_exceeded"] = True
                self.client.cancel_goal(goal_id)

        def result(status, value):
            nav.update(
                status=status,
                error_code=value.get("error_code", 0),
                result=value,
                wall_time=time.time(),
            )
            event("nav2_result", **nav)
            done.set()

        try:
            self.fresh()
            if self.stopping.is_set():
                raise RuntimeError("daemon stop latch is set")
            with self.goal_lock:
                self.goal_id = goal_id
            goal = {
                "pose": {
                    "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                    "pose": {
                        "position": {
                            "x": float(site["x"]),
                            "y": float(site["y"]),
                            "z": 0.0,
                        },
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(site["yaw"] / 2),
                            "w": math.cos(site["yaw"] / 2),
                        },
                    },
                },
                "behavior_tree": "",
            }
            event("nav2_request", site_id=site_id, goal=goal)
            self.client.send_goal(
                action="/navigate_to_pose",
                action_type="nav2_msgs/action/NavigateToPose",
                args=goal,
                goal_id=goal_id,
                on_feedback=feedback,
                on_result=result,
            )
            while time.monotonic() < deadline:
                sample = self.fresh()
                if not samples or sample["sequence"] != samples[-1]["sequence"]:
                    if not samples or sample["sim_time"] > samples[-1]["sim_time"]:
                        last_sim_change = time.monotonic()
                    samples.append(sample)
                scan = self.sensor.snapshot()
                if (not scan or not scan.get("valid")
                    or not 0 <= time.time() - scan["wall_time"] <= 3
                    or abs(sample["sim_time"] - scan["stamp"]) > 1):
                    raise RuntimeError("LiDAR stream is stale or invalid during action")
                if time.monotonic() - last_sim_change > 2:
                    raise RuntimeError("simulation clock did not advance")
                if self.stopping.is_set() or nav.get("recovery_limit_exceeded"):
                    raise RuntimeError("stop or bounded recovery limit reached")
                if done.is_set():
                    if nav.get("status") != 4 or nav.get("error_code") != 0:
                        raise RuntimeError("Nav2 terminal result was not SUCCEEDED")
                    proof = verify_visit(
                        samples,
                        site,
                        nav,
                        self.sensor.snapshot(),
                        body_path=self.body_path,
                    )
                    if proof["success"]:
                        break
                    if time.time() - nav["wall_time"] > 20:
                        raise RuntimeError(
                            "arrival/dwell/LiDAR verification did not pass"
                        )
                time.sleep(0.1)
            else:
                raise TimeoutError("single-site deadline reached")
            data = {
                "schema_version": "rosclaw.isaac_visit.v1",
                "action_id": action.action_id,
                "body_id": action.body_id,
                "body_snapshot_hash": action.body_snapshot_hash,
                "site_id": site_id,
                "site": site,
                "started_wall_time": started,
                "finished_wall_time": time.time(),
                "nav2": nav,
                "verification": proof,
                "physics_source": str(self.physics.parent),
                "trajectory": samples,
                "events": events,
            }
            write_json_atomic(path, data)
            artifact = {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            return self.result(
                ActionState.COMPLETED,
                proof={**proof, "site_id": site_id, "evidence_artifact": artifact},
                artifacts=[],  # A visit is evidence, not the whole mission deliverable.
            )
        except Exception as exc:
            cancel_errors = []
            try:
                self.client.cancel_goal(goal_id)
            except Exception as cancel_error:
                cancel_errors.append(str(cancel_error))
            if not done.wait(5) or nav.get("status") == 6:
                try:
                    fallback = subprocess.run(
                        [str(Path(self.config["challenge"]) / "scripts/ros-container.sh"),
                         "python3", "/lab/ros2/cancel_navigation.py", "--timeout", "5"],
                        capture_output=True, text=True, timeout=12,
                        env={**__import__("os").environ,
                             "ROSCLAW_CONTAINER_NAME": "rosclaw-warehouse-emergency-cancel"},
                    )
                    event("DDS_cancel_fallback", returncode=fallback.returncode,
                          stdout=fallback.stdout, stderr=fallback.stderr)
                except Exception as cancel_error:
                    cancel_errors.append(str(cancel_error))
            stop_verified = False
            stop_deadline = time.monotonic() + 3
            while time.monotonic() < stop_deadline:
                try:
                    sample = self.fresh()
                    if not samples or samples[-1]["sequence"] != sample["sequence"]:
                        samples.append(sample)
                    stop_verified = (math.hypot(*sample["linear_velocity_xyz"][:2]) <= 0.02
                                     and abs(sample["angular_velocity_xyz"][2]) <= 0.1)
                    if stop_verified:
                        break
                except Exception:
                    break
                time.sleep(0.1)
            # Acknowledged cancellation alone is never physical-stop proof.
            failure = {
                "action_id": action.action_id,
                "site_id": site_id,
                "status": "FAIL",
                "error": str(exc),
                "physical_stop_verified": stop_verified,
                "cancel_errors": cancel_errors,
                "nav2": nav,
                "trajectory": samples,
                "events": events,
            }
            write_json_atomic(path, failure)
            return self.result(
                ActionState.FAILED,
                error=str(exc),
                proof={"failure_artifact": str(path)},
            )
        finally:
            with self.goal_lock:
                self.goal_id = None

    def result(self, state, *, error=None, proof=None, artifacts=None):
        return ActionExecutionResult(
            final_state=state,
            evidence_level=EvidenceLevel.TASK_VERIFIED
            if state is ActionState.COMPLETED
            else EvidenceLevel.REQUESTED,
            evidence_domain=EvidenceDomain.SIMULATION,
            policy_decision={
                "allowed": state is not ActionState.BLOCKED,
                "reason": "registered_isaac_SIM_only",
            },
            simulation_result={"engine": "isaacsim_physx", "has_physics": True},
            dispatch_result={
                "owner": self.owner,
                "accepted": state is not ActionState.BLOCKED,
            },
            verification_result=proof or {},
            artifacts=artifacts or [],
            errors=[{"code": "ISAAC_NAV_FAILED", "message": error}] if error else [],
        )
