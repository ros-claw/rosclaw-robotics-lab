"""Compare real camera probe captures from separate simulation resets."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", required=True)
    args = parser.parse_args()
    runs = json.loads(args.validation.read_text())
    if len(runs) != 3 or any(row["status"] != "PASS" for row in runs):
        raise ValueError("Three passing real camera probes required")
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
    names = {
        "robot": "Robot forward view (display camera, not a Hawk ROS image topic)",
        "follow": "Third-person chase camera | 3 m behind, 1.4 m above floor",
        "top-close": "Tracking top view | 12 m width (original overview: 48 m)",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen([
        args.ffmpeg, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", "1920x1080", "-r", "8", "-i", "-", "-an",
        "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(args.output),
    ], stdin=subprocess.PIPE)
    sources = []
    try:
        for row in runs:
            root = Path(row["run_directory"])
            nav = [json.loads(line) for line in (root / "camera-nav2.jsonl").read_text().splitlines()]
            frames = [json.loads(line) for line in (root / "baseline-frames/timestamps.jsonl").read_text().splitlines()]
            start = nav[0]["wall_time"] + 5
            end = min(start + 60, frames[-1]["wall_time"] - 0.1)
            compression = (end - start) / 15
            if compression <= 0:
                raise ValueError("No captured motion interval")
            if frames[0]["wall_time"] > start or frames[-1]["wall_time"] < end:
                raise ValueError("Actual captures must cover the chosen wall interval")
            sources.append({"view": row["view"], "run_directory": str(root), "wall_interval": [start, end], "wall_time_compression": compression})
            for index in range(15 * 8):
                wall = start + index / 8 * compression
                frame = min(frames, key=lambda item: abs(item["wall_time"] - wall))
                with Image.open(root / "baseline-frames" / frame["frame"]) as source:
                    if source.size != (1920, 1080):
                        raise ValueError("Native 1920x1080 captures required; no upscaling")
                    canvas = source.convert("RGB")
                draw = ImageDraw.Draw(canvas)
                draw.rectangle((0, 0, 1920, 92), fill="#08121e")
                draw.text((22, 8), names[row["view"]], font=font, fill="white")
                draw.text((22, 47), f"REAL ISAAC RENDER | 1080p | {compression:.2f}x wall-time | separate Nav2 probes; not Native patrol", font=font, fill="#75d9ff")
                encoder.stdin.write(canvas.tobytes())
    finally:
        encoder.stdin.close()
        result = encoder.wait()
    if result:
        raise RuntimeError(f"FFmpeg failed: {result}")
    args.output.with_suffix(".json").write_text(json.dumps({
        "scope": "Camera validation only; three independent single-goal Nav2 probes",
        "duration_seconds": 45, "fps": 8, "native_resolution": [1920, 1080],
        "sources": sources, "audio": False,
        "source": "actual timestamped Isaac viewport PNG captures; no synthetic views or interpolation",
        "renderer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }, indent=2) + "\n")


if __name__ == "__main__":
    main()
