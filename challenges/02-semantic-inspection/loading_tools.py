"""Bounded robot-facing prior/proposals/measured feedback only. No scene truth."""

from collections import Counter
import argparse
import json
from pathlib import Path
from mcp.server.fastmcp import FastMCP

p = argparse.ArgumentParser()
p.add_argument("--directory", type=Path, required=True)
a = p.parse_args()
root = a.directory.resolve()
server = FastMCP("warehouse-loading-inspection")


@server.tool(name="loading.observe_scene")
def observe_scene() -> dict:
    """Read facilities and shelf distances, safe observation proposals and actual measured coverage. No motion. Choose the forklift near shelves, not near robot. UNKNOWN can require a second view (max 2)."""
    c = json.loads((root / "loading-catalog.json").read_text())
    targets = []
    for row in c["targets"]:
        targets.append(
            {
                **{k: v for k, v in row.items() if k not in {"rejections"}},
                "rejection_reasons": dict(
                    Counter(r["reason"] for r in row["rejections"]).most_common(3)
                ),
            }
        )
    summary = c["latest_inspection"]
    if summary:
        summary = {
            k: v
            for k, v in summary.items()
            if k
            not in {
                "free_cells",
                "occupied_cells",
                "region",
                "thresholds",
                "clearance_halo",
                "clearance_safe_center_cells",
                "clearance_blocked_center_cells",
                "height_band_mask_this_view",
                "accumulated_height_band_mask",
            }
        }
    result = {**c, "targets": targets, "latest_inspection": summary}
    if len(json.dumps(result, ensure_ascii=False)) > 7600:
        raise ValueError("Bounded loading view exceeds budget")
    return result


@server.tool(name="navigation.navigate_to_pose")
def navigate(proposal_id: str) -> dict:
    """Request ONE immutable safe observation or actual-start return proposal through rosclawd."""
    raise RuntimeError("Action requires existing Agentd rosclawd channel")


@server.tool(name="mission.verify_and_remember")
def remember(action_ids: list[str], inspection_report: dict) -> dict:
    """After return, submit ordered canonical IDs and your evidence-based report. Exact report keys: target_id, result(CLEAR/OBSTRUCTED/UNKNOWN), extra_obstacle_detected(bool), uncertainties(list[str]), summary(str). Verifies physical evidence and persists existing Memory."""
    raise RuntimeError("Memory requires existing Agentd rosclawd channel")


for tool in server._tool_manager._tools.values():
    tool.parameters["additionalProperties"] = False
server.run()
