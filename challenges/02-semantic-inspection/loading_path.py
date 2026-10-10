"""Read-only candidate path previews before the Agent selects a proposal."""

import subprocess
import time
from safety import canonical_hash, swept_footprint


def validate_catalog_proposals(proposals, entries):
    """Never expose retired IDs or content that differs from daemon registration."""
    seen = set()
    for proposal in proposals:
        ident = proposal["proposal_id"]
        if ident in seen:
            raise ValueError("Catalog contains duplicate proposal IDs")
        seen.add(ident)
        entry = entries.get(ident)
        if entry is None:
            raise ValueError("Catalog contains an unregistered proposal ID")
        candidate = entry["candidate"]
        raw = {k: v for k, v in candidate.items() if k != "proposal_id"}
        if canonical_hash({"candidate": raw, "evidence": entry["evidence"]}) != ident:
            raise ValueError("Catalog proposal hash differs from registered evidence")
        if any(proposal[k] != candidate[k] for k in candidate):
            raise ValueError("Catalog proposal content differs from registered target")
    return proposals


def preview_candidates(
    candidates, entries, probe, footprint, *, now=time.time, limit=3, on_record=None
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
        if on_record is not None:
            on_record(record)
        if len(accepted) == limit:
            break
    return accepted, records


def verify_refused_candidates(records, expected_positions, footprint, target):
    """Recheck every generated path; a single safe or missing candidate defeats refusal."""
    rejected_xy = set()
    for record in records:
        candidate = record["proposal"]["candidate"]
        if record["status"] != "REJECTED" or candidate["target_prim"] != target:
            raise ValueError("Refusal requires actual rejected target paths")
        try:
            swept_footprint(
                record["checked_path"],
                record["snapshot"]["costmap"],
                footprint,
                padding=0.08,
            )
        except ValueError:
            rejected_xy.add((candidate["x"], candidate["y"]))
        else:
            raise ValueError("Refusal contains a safe observation path")
    if rejected_xy != set(expected_positions):
        raise ValueError("Refusal did not examine every generated observation path")
    return True
