"""Read-only candidate path previews before the Agent selects a proposal."""

import subprocess
import time
from safety import swept_footprint


def preview_candidates(
    candidates, entries, probe, footprint, *, now=time.time, limit=3
):
    accepted, records = [], []
    for candidate in candidates:
        record = {
            "proposal": entries[candidate["proposal_id"]],
            "role": "Actual Nav2 full-footprint prefilter; never motion authorization",
        }
        try:
            snapshot = probe(candidate)
            record["snapshot"] = snapshot
            path = [
                snapshot["base_pose"],
                *snapshot["path"],
                [candidate[k] for k in ("x", "y", "yaw")],
            ]
            record["checked_path"] = path
            record["swept_footprint"] = swept_footprint(
                path, snapshot["costmap"], footprint, padding=0.08
            )
            if not 0 <= now() - snapshot["wall_time"] <= 3:
                raise ValueError("Candidate path preview is stale")
            record["status"] = "PASS"
            accepted.append(candidate)
        except (ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            record.update(status="REJECTED", error=str(exc))
        records.append(record)
        if len(accepted) == limit:
            break
    return accepted, records
