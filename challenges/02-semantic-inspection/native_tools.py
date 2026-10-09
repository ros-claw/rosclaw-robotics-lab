"""Native can observe daemon-issued proposals; physical authority remains daemon-owned."""

import argparse
import json
from pathlib import Path
from mcp.server.fastmcp import FastMCP

p = argparse.ArgumentParser()
p.add_argument("--directory", type=Path, required=True)
a = p.parse_args()
root = a.directory.resolve()
config = json.loads((root / "execution_config.json").read_text())
server = FastMCP("isaac-semantic-observation")


@server.tool(name="semantic.observe_candidates")
def observe_candidates() -> dict:
    """Read USD-known development shelves, daemon-generated safe proposals and actual current robot state. No motion."""
    catalog = json.loads((root / "semantic-catalog.json").read_text())
    return {
        **catalog,
        "physics": json.loads(
            (Path(config["physics_directory"]) / "physics-latest.json").read_text()
        ),
        "requirements": [
            "Choose the shelf requested by the user from the USD-known catalog yourself; no fixed four-site IDs are used.",
            "Choose one fresh proposal_id and request exactly one navigation.navigate_to_pose at a time.",
            "Observe candidates again before each navigation; proposals expire. Use the return_to_initial_pose proposal to return.",
            "After the requested shelf and return, call mission.verify_and_remember with their canonical action_ids in order.",
            "Register the verified artifact and close the existing TaskKernel only after the final canonical verification receipt.",
            "Inspection scope is explicit. USD supplies semantics; display cameras are not Agent sensor observations.",
        ],
        "authorization": False,
        "evidence_domain": "SIMULATION",
    }


@server.tool(name="navigation.navigate_to_pose")
def navigate(proposal_id: str) -> dict:
    """Request ONE immutable daemon-generated observation/return proposal through rosclawd. No arbitrary coordinates."""
    raise RuntimeError("Physical actions require Agentd rosclawd action channel")


@server.tool(name="mission.verify_and_remember")
def remember(action_ids: list[str]) -> dict:
    """Replay canonical observation and return evidence, verify task semantics and store existing Practice/Memory."""
    raise RuntimeError("Persistence requires Agentd rosclawd action channel")


for tool in server._tool_manager._tools.values():
    tool.parameters["additionalProperties"] = False
server.run()
