"""Compile measured USD Body and isolated Native SIM declarations."""

import argparse
import base64
import time
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path
import yaml
from rosclaw.body.compiler import compute_checksum
from rosclaw.body.resolver import BodyResolver
from rosclaw.body.schema import BodyYaml, CalibrationYaml, EurdfProfile


def prepare(root, challenge, physics, task, order, credentials, require_obstacle=False, scenario=None):
    root.mkdir(parents=True, exist_ok=False)
    os.chmod(root, 0o700)
    home = root / "home"
    home.mkdir()
    inventory_path = physics / "stage-inventory.json"
    inventory = json.loads(inventory_path.read_text())
    sites = yaml.safe_load((challenge / "config/inspection_sites.yaml").read_text())
    profile = EurdfProfile(
        profile_id="nova_carter_isaac_6_1",
        profile_version="1.0.0",
        vendor="NVIDIA",
        model="Nova Carter ROS",
        display_name="Measured NVIDIA Nova Carter in Isaac Sim",
        description="USD inventory-derived SIM Body; no invented URDF or REAL authority.",
        assets={
            "usd": "Assets/Isaac/6.1/Isaac/Robots/NVIDIA/NovaCarter/Variants/Sensors/nova_carter_sensors.usd"
        },
        identity={"robot_class": "mobile_base"},
        frames={"root": "base_link", "base": "base_link", "map": "map", "odom": "odom"},
        joints=inventory["robot_joints"],
        sensors=[
            {
                "name": "front_3d_lidar",
                "type": "lidar",
                "parent_link": "front_3d_lidar",
                "topic": "/scan",
            }
        ],
        actuators=[
            {"name": n, "type": "wheel_motor", "joint": n}
            for n in ["joint_wheel_left", "joint_wheel_right"]
        ],
        provider_interfaces={
            "ros_capability_bindings": {
                "navigation.navigate_to_pose": {
                    "name": "/navigate_to_pose",
                    "action_type": "nav2_msgs/action/NavigateToPose",
                    "goal_frame": "map",
                },
                "odometry.observe": {
                    "topic": "/chassis/odom",
                    "message_type": "nav_msgs/msg/Odometry",
                },
                "lidar.observe": {
                    "topic": "/scan",
                    "message_type": "sensor_msgs/msg/LaserScan",
                },
            }
        },
        capability_hints={
            "all": ["navigation.navigate_to_pose", "patrol.verify_and_remember"]
        },
        safety={
            "safety_level": "STRICT",
            "environment": {"real_robot_execution_allowed": False},
            "motion_limits": {
                "max_linear_velocity_mps": 0.8,
                "max_angular_velocity_rps": 1.2,
            },
            "navigation_contract": {
                "frame_id": "map",
                "registered_site_ids": list(sites["sites"]),
                "position_tolerance_m": 0.4,
                "stable_dwell_sim_seconds": 2.0,
                "single_goal_timeout_seconds": 420,
                "maximum_recoveries": 6,
                "required_non_floor_collisions": 0,
            },
            "stop_policy": {
                "clock_pause": "cancel_and_fail",
                "observer_stale": "cancel_and_fail",
                "body_mismatch": "block",
                "unknown_collision_state": "block",
            },
        },
        sandbox={"compatible_engines": ["isaacsim"], "preferred_engine": "isaacsim"},
        metadata={
            "evidence_domain": "SIMULATION",
            "inventory_sha256": hashlib.sha256(inventory_path.read_bytes()).hexdigest(),
            "map_sha256": sites["map_sha256"],
            "physics_body_path": "/World/Nova_Carter_ROS/chassis_link",
            "nav2_footprint": [
                [0.14, 0.25],
                [0.14, -0.25],
                [-0.607, -0.25],
                [-0.607, 0.25],
            ],
            "max_linear_velocity_mps": 0.8,
            "max_angular_velocity_rps": 1.2,
        },
    )
    if scenario is not None and scenario.get("raw_pointcloud"):
        profile.sensors.append({"name": "front_3d_lidar_points", "type": "lidar", "parent_link": "front_3d_lidar", "topic": "/front_3d_lidar/lidar_points"})
        profile.provider_interfaces["ros_capability_bindings"]["pointcloud.observe"] = {
            "topic": "/front_3d_lidar/lidar_points", "message_type": "sensor_msgs/msg/PointCloud2"}
    if scenario is not None:
        profile.safety["navigation_contract"] = scenario["navigation_contract"]
        profile.capability_hints["all"] = ["navigation.navigate_to_pose", "mission.verify_and_remember"]
    resolver = BodyResolver(workspace=home)
    resolver.ensure_body_dir()
    resolver.eurdf_profile_path.write_text(
        yaml.safe_dump(profile.to_dict(), sort_keys=False)
    )
    checksum = compute_checksum(resolver.eurdf_profile_path)
    uri = "rosclaw://eurdf/nova_carter_isaac_6_1@1.0.0"
    resolver.eurdf_lock_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": "rosclaw.eurdf_lock.v1",
                "profile_id": profile.profile_id,
                "profile_version": "1.0.0",
                "uri": uri,
                "source": "pinned_vendor_simulation",
                "checksum": checksum,
            }
        )
    )
    body = BodyYaml(
        body_instance={"id": "nova_carter_isaac", "robot_model": profile.profile_id},
        model_ref={"eurdf_uri": uri, "checksum": checksum},
        installed_components={
            kind: {
                c["name"]: {"installed": True, "status": "available"}
                for c in components
            }
            for kind, components in [
                ("sensors", profile.sensors),
                ("actuators", profile.actuators),
            ]
        },
        agent_policy={"direct_real_robot_execution_allowed": False},
        metadata={"evidence_domain": "SIMULATION", "source": "composed_USD_inventory"},
    )
    resolver.body_yaml_path.write_text(yaml.safe_dump(body.to_dict(), sort_keys=False))
    resolver.calibration_yaml_path.write_text(
        yaml.safe_dump(CalibrationYaml().to_dict())
    )
    effective = resolver.recompile_effective_body()
    config = {
        "body_id": effective.body_instance_id,
        "body_snapshot_hash": effective.compute_hash(),
        "maximum_recoveries": 6,
        "require_obstacle_evidence": require_obstacle,
        "physics_directory": str(physics),
        "physics_body_path": "/World/Nova_Carter_ROS/chassis_link",
        "sites": sites["sites"],
        "task": task,
        "expected_order": order,
        "mission_id": root.name,
        "challenge": str(challenge),
    }
    if scenario is not None:
        config.update(scenario["config"])
        config["sites"] = {}
    if require_obstacle:
        map_proof = json.loads((challenge / "reports/obstacle-map-validation.json").read_text())
        actual_map = Path(os.environ["ISAAC_ROS_WS"]) / "src/navigation/carter_navigation/maps/carter_warehouse_navigation.png"
        if (hashlib.sha256(actual_map.read_bytes()).hexdigest() != map_proof["map_sha256"]
            or map_proof["map_sha256"] != sites["map_sha256"] or not map_proof["all_box_cells_free"]):
            raise ValueError("unmapped obstacle static-map validation differs")
        config["obstacle_static_map_validation"] = map_proof
    (root / "execution_config.json").write_text(json.dumps(config, indent=2) + "\n")
    (root / "body.json").write_text(
        json.dumps(
            {
                "body_id": config["body_id"],
                "effective_body_hash": config["body_snapshot_hash"],
            }
        )
        + "\n"
    )
    schema = {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            k: {"type": t}
            for k, t in {
                "snapshot_id": "string",
                "snapshot_hash": "string",
                "captured_at": "string",
                "system": "string",
                "diagnosis": "object",
                "sites": "object",
                "requirements": "array",
                "physics": "object",
                "evidence_domain": "string",
                "authorization": "boolean",
            }.items()
        },
    }
    schema["required"] = list(schema["properties"])
    native = {
        "agent": {
            "enabled": True,
            "body_id": config["body_id"],
            "default_mode": "SIMULATION",
            "default_profile": "embodied_default",
            "max_tool_rounds": 30,
            "ros_expert": {"enabled": True, "endpoint": "ws://127.0.0.1:19091"},
        },
        "mcp_servers": [
            {
                "name": "isaac-patrol",
                "command": sys.executable,
                "args": [
                    str(challenge / "rosclaw/native_tools.py"),
                    "--directory",
                    str(root),
                ],
                "env_refs": ["ROSCLAW_HOME"],
                "observation_tools": ["ros.observe_system"],
                "action_tools": [
                    "navigation.navigate_to_pose",
                    "patrol.verify_and_remember",
                ],
                "supported_modes": ["SIMULATION"],
                "required_body_types": [config["body_id"]],
                "effect_domain": "ROS",
                "timeout_ms": 450000,
                "output_schemas": {"ros.observe_system": schema},
            }
        ],
    }
    if scenario is not None:
        native["mcp_servers"][0].update(scenario["mcp"])
    (home / "config.yaml").write_text(yaml.safe_dump(native, sort_keys=False))
    if credentials:
        dest = home / "agent"
        dest.mkdir(mode=0o700)
        for name in ["settings.json", "models.json", "models-store.json", "auth.json"]:
            source = Path.home() / ".rosclaw/agent" / name
            if source.is_file():
                shutil.copyfile(source, dest / name)
                os.chmod(dest / name, 0o600)
    if credentials:
        settings = json.loads((dest / "settings.json").read_text())
        # An isolated fixture may inherit an expired access-only ROSClaw login.
        # Reuse a current local Codex access token only for the same account.
        # Never copy refresh tokens or modify either global authentication file.
        auth_path = dest / "auth.json"
        auth = json.loads(auth_path.read_text())
        login = auth.get("openai-codex", {})
        if settings.get("defaultProvider") == "openai-codex" and login.get("expires", 0) <= time.time() * 1000:
            codex_path = Path.home() / ".codex/auth.json"
            if not codex_path.is_file():
                raise RuntimeError("Configured Codex login expired; no current local login")
            tokens = json.loads(codex_path.read_text()).get("tokens", {})
            if tokens.get("account_id") != login.get("accountId"):
                raise RuntimeError("Current Codex login differs from configured ROSClaw account")
            token = tokens["access_token"]
            part = token.split(".")[1]
            expiry = json.loads(base64.urlsafe_b64decode(part + "=" * (-len(part) % 4)))["exp"]
            if expiry - time.time() < 1800:
                raise RuntimeError("Current local Codex access token lacks a 30-minute run window")
            login.update(access=token, expires=expiry * 1000, refresh="")
            auth_path.write_text(json.dumps(auth, indent=2) + "\n")
            os.chmod(auth_path, 0o600)
        settings["hideThinkingBlock"] = True
        (dest / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
        store = json.loads((dest / "models-store.json").read_text())
        model = next(
            m
            for m in store[settings["defaultProvider"]]["models"]
            if m["id"] == settings["defaultModel"]
        )
        declarations = json.loads((dest / "models.json").read_text())
        declarations.setdefault("providers", {})[settings["defaultProvider"]] = {
            "api": model["api"],
            "baseUrl": model["baseUrl"],
            "models": [model],
        }
        (dest / "models.json").write_text(json.dumps(declarations, indent=2) + "\n")
    if scenario is not None:
        shutil.copyfile(inventory_path, root / "semantic-inventory.json")
        (root / "body-effective.json").write_text(json.dumps(effective.to_dict(), indent=2) + "\n")
    print(
        json.dumps(
            {
                "status": "PREPARED",
                "directory": str(root),
                "body_id": config["body_id"],
                "body_snapshot_hash": config["body_snapshot_hash"],
            }
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--physics", type=Path, required=True)
    p.add_argument("--task", required=True)
    p.add_argument(
        "--expected-order",
        nargs="+",
        required=True,
        choices=["entry", "shelf", "aisle", "home"],
    )
    p.add_argument("--use-configured-model", action="store_true")
    p.add_argument("--require-obstacle-evidence", action="store_true")
    a = p.parse_args()
    prepare(
        a.directory.resolve(),
        Path(__file__).resolve().parents[1],
        a.physics.resolve(),
        a.task,
        a.expected_order,
        a.use_configured_model,
        a.require_obstacle_evidence,
    )
