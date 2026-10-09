"""Check actual three-camera frames cover the entire Native execution window."""

import json
import math
from pathlib import Path


def verify_recording(directory: Path, started: float, finished: float) -> dict:
    # Recording failure must remain separate from physical task acceptance.
    try:
        return _verify_recording(directory, started, finished)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {
            "status": "FAIL",
            "failures": ["Unreadable recording index: " + str(exc)],
        }


def _verify_recording(directory: Path, started: float, finished: float) -> dict:
    failures = []
    rows = [
        json.loads(line)
        for line in (directory / "timestamps.jsonl").read_text().splitlines()
    ]
    if (
        not all(
            math.isfinite(t)
            for t in [started, finished] + [r["wall_time"] for r in rows]
        )
        or started > finished
    ):
        raise ValueError("Invalid recording or Native timestamps")
    if not rows or rows[0]["wall_time"] > started or rows[-1]["wall_time"] < finished:
        failures.append("Frames do not cover the complete Native execution window")
    for row in rows:
        views = row.get("views", {})
        if set(views) != {"follow", "robot", "top-close"}:
            failures.append("Missing camera")
            continue
        ids = [view.get("render_frame") for view in views.values()]
        if None in ids or len(set(ids)) != 1:
            failures.append("Cameras were not captured on the same renderer frame")
        for view in views.values():
            path = (directory / view["frame"]).resolve()
            if not path.is_relative_to(directory.resolve()) or not path.is_file():
                failures.append("Missing or escaping camera file")
    gaps = [b["wall_time"] - a["wall_time"] for a, b in zip(rows, rows[1:])]
    if any(gap <= 0 or gap > 10 for gap in gaps):
        failures.append(
            "Capture timestamps are unordered or have a gap over 10 seconds"
        )
    return {
        "status": "FAIL" if failures else "PASS",
        "failures": sorted(set(failures)),
        "frame_groups": len(rows),
        "native_wall_time": [started, finished],
        "capture_wall_time": [rows[0]["wall_time"], rows[-1]["wall_time"]]
        if rows
        else [],
        "max_gap_wall_seconds": max(gaps, default=0),
        "interpolation": False,
    }
