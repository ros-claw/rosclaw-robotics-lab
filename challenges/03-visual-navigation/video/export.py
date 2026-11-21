"""Encode live Isaac frames using their recorded wall-time intervals."""

import argparse
import json
from pathlib import Path
import re
import subprocess


def concat_manifest(rows):
    if len(rows) < 2:
        raise ValueError("at least two live frames are required")
    lines = ["ffconcat version 1.0"]
    for index, row in enumerate(rows):
        name = row["name"]
        if not re.fullmatch(r"[0-9]+\.jpg", name):
            raise ValueError("invalid captured frame name")
        lines.extend((f"file {name}", "option framerate 1000"))
        if index + 1 < len(rows):
            interval = (int(rows[index + 1]["host_mono_ns"]) - int(row["host_mono_ns"])) / 1e9
            if interval < 0.001:
                raise ValueError("frame arrival times must advance by at least a millisecond")
            lines.append(f"duration {interval:.9f}")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    directory = args.capture.resolve()
    rows = [json.loads(line) for line in (directory / "frames.jsonl").read_text().splitlines()]
    listing = directory / "wall-time.ffconcat"
    listing.write_text(concat_manifest(rows))
    subprocess.run(["ffmpeg", "-nostdin", "-n", "-hide_banner", "-loglevel", "warning",
                    "-safe", "0", "-i", str(listing), "-fps_mode", "vfr",
                    "-c:v", "libx264", "-threads", "2", "-preset", "fast", "-crf", "18",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    str(directory / "isaac-live-1x.mp4")], check=True)


if __name__ == "__main__":
    main()
