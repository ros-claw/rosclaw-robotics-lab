"""Native catalog and live read-only observation; motion belongs to rosclawd."""

import argparse
import json
from pathlib import Path
from typing import Literal
from mcp.server.fastmcp import FastMCP
from rosclaw.connectors.ros.context.body import configured_ros_body
from rosclaw.connectors.ros.context.compiler import compile_agent_summary
from rosclaw.connectors.ros.context.probe_client import read_snapshot
from rosclaw.connectors.ros.diagnosis import diagnose

p = argparse.ArgumentParser()
p.add_argument("--directory", type=Path, required=True)
a = p.parse_args()
config = json.loads((a.directory / "execution_config.json").read_text())
server = FastMCP("isaac-warehouse-patrol")


@server.tool(name="ros.observe_system")
def observe_system() -> dict:
    """Read live ROS graph/time/TF/Nav2 readiness, measured sites and independent physics."""
    body = configured_ros_body(config["body_id"], config["body_snapshot_hash"])
    model = read_snapshot(
        endpoint="ws://127.0.0.1:19091", robot_id=config["body_id"], body=body
    )
    return {
        "snapshot_id": model.snapshot_id,
        "snapshot_hash": model.snapshot_hash,
        "captured_at": model.captured_at.isoformat(),
        "system": compile_agent_summary(model),
        "diagnosis": diagnose(model),
        "sites": config["sites"],
        "requirements": [
            "Follow the user task, selecting each requested site yourself. Every navigation call visits exactly ONE site.",
            "Every visit requires Nav2 SUCCEEDED, independently measured position error <=0.4m, stable dwell >=2 simulation seconds, fresh LiDAR, and zero non-floor collisions.",
            "After all requested visits and Home, call patrol.verify_and_remember with their exact canonical action_ids in execution order.",
            "Register the verified deliverable and close the existing TaskKernel task before declaring completion.",
        ],
        "physics": json.loads(
            (Path(config["physics_directory"]) / "physics-latest.json").read_text()
        ),
        "evidence_domain": "SIMULATION",
        "authorization": False,
    }


@server.tool(name="navigation.navigate_to_pose")
def navigate(site_id: Literal["entry", "shelf", "aisle", "home"]) -> dict:
    """Navigate to ONE observed site through daemon; verify arrival, dwell, scan and contacts. This tool executes no sequence."""
    raise RuntimeError("physical actions require the Agentd rosclawd action channel")


@server.tool(name="patrol.verify_and_remember")
def remember(action_ids: list[str]) -> dict:
    """Verify ordered canonical visit receipts against operator task; persist existing Practice and Memory. Does not move robot."""
    raise RuntimeError("persistence requires the Agentd rosclawd action channel")


for tool in server._tool_manager._tools.values():
    tool.parameters["additionalProperties"] = False
server.run()
