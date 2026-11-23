"""Per-episode requests for a fresh frame from the persistent ROS subscriber."""

import fcntl
import json
import re
import time


def request_directory(root, request, now_ns):
    try:
        name = request["capture"]
        start, end = request["requested_ns"], request["deadline_ns"]
        if re.fullmatch(r"rgbd-[0-9a-f]{8}", name) and start <= now_ns < end:
            return root / name
    except (KeyError, TypeError):
        pass
    return None


def eligible_frame(received_ns, requested_ns, stamp_ns, clock_ns):
    return received_ns >= requested_ns and 0 <= clock_ns - stamp_ns <= 250_000_000


def write_atomic(path, content):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(content, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def request_frame(run, destination, timeout):
    deadline = time.monotonic_ns() + round(timeout * 1e9)
    with (run / "capture-request.lock").open("a") as lock:
        while time.monotonic_ns() < deadline:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                time.sleep(0.025)
        else:
            raise RuntimeError("persistent RGB-D capture was busy")
        requested = time.monotonic_ns()
        write_atomic(run / "capture-request.json",
                     {"capture": destination.name, "requested_ns": requested, "deadline_ns": deadline})
        result = destination / "observation_summary.json"
        while time.monotonic_ns() < deadline:
            if result.is_file():
                return json.loads(result.read_text())
            time.sleep(0.015)
    raise RuntimeError(f"persistent RGB-D capture missed its deadline. See {destination}")
