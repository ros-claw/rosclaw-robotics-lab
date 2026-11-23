"""Check the two authored RGB-D calibration plates against live capture evidence."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.geometry import CameraIntrinsics


def verify(directory):
    observations = [json.loads(line) for line in (directory / "observations.jsonl").read_text().splitlines()]
    if len(observations) < 2:
        raise ValueError("At least two live calibration frames are required")
    measurements = []
    last = -1
    for observation in observations:
        if observation["source"] != "isaac-live-calibration":
            raise ValueError("Input is not a live calibration frame")
        stamp = int(observation["stamp_ns"])
        if stamp <= last:
            raise ValueError("Calibration frames are not unique and time-ordered")
        last = stamp
        camera = observation["camera"]
        k = camera["k"]
        intrinsics = CameraIntrinsics(camera["width"], camera["height"], k[0], k[4], k[2], k[5])
        for field in ("rgb", "depth"):
            image = observation[field]
            if hashlib.sha256((directory / image["ref"]).read_bytes()).hexdigest() != image["sha256"]:
                raise ValueError("Capture hash mismatch")
        depth = np.load(directory / observation["depth"]["ref"], allow_pickle=False)
        for target in ((-0.20, -0.12, 1.0), (0.32, 0.16, 1.6)):
            u, v = map(round, intrinsics.project(target))
            sample = float(depth[v, u])
            actual = intrinsics.unproject(u, v, sample)
            error = float(np.linalg.norm(np.asarray(actual) - target))
            measurements.append({"stamp_ns": str(stamp), "pixel": [u, v],
                                 "expected_xyz_m": target, "sampled_xyz_m": actual,
                                 "error_m": error, "pass": error <= 0.01})
    return {"pass": all(item["pass"] for item in measurements), "measurements": measurements}


if __name__ == "__main__":
    result = verify(Path(sys.argv[1]))
    print(json.dumps(result, indent=2, allow_nan=False))
    sys.exit(0 if result["pass"] else 1)
