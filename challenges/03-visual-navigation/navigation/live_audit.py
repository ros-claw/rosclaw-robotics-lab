"""Associate asynchronous vision reviews with the exact active motion goal."""

import json
import math
from subprocess import TimeoutExpired
import threading
import time

from navigation.luna_probe import load_frame, probe


def review_is_current(review, episode, goal_id, gate, sim_ns, wall_ns, position, yaw):
    try:
        return (review["episode_id"] == episode and review["goal_id"] == goal_id
                and review["gate"] == gate
                and 0 <= sim_ns - review["sim_stamp_ns"] <= 8_000_000_000
                and 0 <= wall_ns - review["frame_wall_ns"] <= 8_000_000_000
                and 0 <= wall_ns - review["reply_wall_ns"] <= 4_000_000_000
                and math.dist(position, review["base_xy"]) <= 1.5
                and abs(math.remainder(yaw - review["base_yaw"], math.tau)) <= 1.15)
    except (KeyError, TypeError, ValueError):
        return False


def active_goal(path):
    if not path.exists():
        return None
    goal = None
    for line in path.read_text().splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event["event"] == "nav2_accepted":
            goal = event["goal_id"]
        elif event["event"] in {"nav2_result", "cancel_requested", "test_failed"}:
            goal = None
    return goal


class MotionAudit:
    def __init__(self, interval, limit, capture, layout, model, key, timeout):
        self.interval = interval
        self.remaining = limit
        self.capture = capture
        self.layout = layout
        self.model = model
        self.key = key
        self.timeout = timeout

    def watch(self, controller, review_file, gate, done):
        next_request = time.monotonic()
        timeline = controller.parent / "motion-audit.jsonl"

        def log(event, **values):
            with timeline.open("a") as stream:
                stream.write(json.dumps({"event": event, "host_mono_ns": str(time.monotonic_ns()),
                                         "gate": gate, "controller": controller.name, **values},
                                        ensure_ascii=False) + "\n")

        while self.remaining and not done.wait(0.2):
            goal = active_goal(controller)
            now = time.monotonic()
            if goal is None or now < next_request:
                continue
            next_request = now + self.interval
            try:
                capture_dir = self.capture()
                if done.is_set() or active_goal(controller) != goal:
                    continue
                observation, _ = load_frame(capture_dir, 1)
                model_dir = capture_dir / "luna-motion-audit"
                self.remaining -= 1
                log("audit_requested", goal_id=goal, model_dir=str(model_dir),
                    sim_stamp_ns=observation["stamp_ns"], image_sha256=observation["rgb"]["sha256"])
                intent = probe(capture_dir, 1, self.layout, model_dir, self.model, self.timeout,
                               self.key, audit_gate=gate)
                current = not done.is_set() and active_goal(controller) == goal
                log("audit_returned", goal_id=goal, model_dir=str(model_dir), current_goal=current,
                    **intent)
                if not current or intent["decision"] != "review":
                    continue
                base = next(tf for tf in observation["transforms"]
                            if tf["target"] == "map" and tf["source"] == "base_link")
                x, y, z, w = base["quaternion_xyzw"]
                review = {"episode_id": observation["episode_id"], "goal_id": goal, "gate": gate,
                          "sim_stamp_ns": int(observation["stamp_ns"]),
                          "frame_wall_ns": int(observation["rgb_receive_mono_ns"]),
                          "reply_wall_ns": time.monotonic_ns(), "base_xy": base["xyz_m"][:2],
                          "base_yaw": math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)),
                          "observation_id": intent["observation_id"]}
                temporary = review_file.with_suffix(".tmp")
                temporary.write_text(json.dumps(review))
                temporary.replace(review_file)
            except (ValueError, OSError, RuntimeError, StopIteration, TimeoutExpired) as error:
                log("audit_failed", goal_id=goal, reason=str(error))

    def start(self, controller, review_file, gate):
        done = threading.Event()
        thread = threading.Thread(target=self.watch, args=(controller, review_file, gate, done),
                                  daemon=True)
        thread.start()
        return done, thread
